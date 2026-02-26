/// FK/IK 変形計算モジュール
/// glam を使ってボーンのワールド座標と回転を計算する
use crate::format::pmx;
use glam::{EulerRot, Mat4, Quat, Vec3};

/// 全ボーンのワールド位置と回転を計算する
///
/// 変形順序（仕様書 □変形順序）:
///   1. transform_order 昇順
///   2. 同値はボーンインデックス順
///   （物理前/後はフラグ読取未実装のため今回除外）
///
/// 変形パラメータ（仕様書 □変形パラメータ）:
///   - ユーザー操作量 (base_rots / base_movs)
///   - 付与回転量/付与移動量 (grant_rots / grant_movs) — IKリンク回転を除く、多重付与用
///   - IKリンク回転量 (local_rots に統合)
///
/// extern_parent_mats: PMXボーンインデックス → 外部親のワールド行列（PMMコンフィグフレームから構築）
///
/// 戻り値: 各ボーンの (world_pos, world_rot) のVec（PMXボーンインデックス順）
pub fn compute_world_transforms(
    pmx_bones: &[pmx::PmxBone],
    movements: &[Vec3],
    rotations: &[Quat],
    ik_enabled: &[bool],
    ik_bone_indices: &[i32],
    extern_parent_mats: &std::collections::HashMap<usize, Mat4>,
) -> Vec<(Vec3, Quat)> {
    let n = pmx_bones.len();
    if n == 0 {
        return Vec::new();
    }

    // transform_order 昇順でソート（同値のときはボーンインデックス順）
    let mut sorted_indices: Vec<usize> = (0..n).collect();
    sorted_indices.sort_by(|&a, &b| {
        pmx_bones[a].transform_order.cmp(&pmx_bones[b].transform_order)
            .then_with(|| a.cmp(&b))
    });

    // ワールド行列を初期化
    let mut world_mats: Vec<Mat4> = vec![Mat4::IDENTITY; n];

    // ユーザー操作量（PMM 入力値）— 付与計算の基底、変更されない
    let base_rots: Vec<Quat> = (0..n)
        .map(|i| rotations.get(i).copied().unwrap_or(Quat::IDENTITY))
        .collect();

    let base_movs: Vec<Vec3> = (0..n)
        .map(|i| movements.get(i).copied().unwrap_or(Vec3::ZERO))
        .collect();

    // 付与回転量/付与移動量（FK後・IKリンク回転を除く）— 多重付与参照用
    // 初期値は base_rots/base_movs（付与なしボーンはこのままになる）
    let mut grant_rots: Vec<Quat> = base_rots.clone();
    let mut grant_movs: Vec<Vec3> = base_movs.clone();

    // 最終ローカル回転/移動（付与+IK後の値、ワールド行列計算に使用）
    let mut local_rots: Vec<Quat> = base_rots.clone();
    let mut local_movs: Vec<Vec3> = base_movs.clone();

    // ── FK 計算（transform_order 昇順）──
    for &i in &sorted_indices {
        let bone = &pmx_bones[i];

        // 付与処理（仕様書 □付与について / □ローカル変形順序）
        if bone.is_add_rotation || bone.is_add_translation {
            if let Some(add_src_idx) = bone.add_bone_index {
                let add_src = add_src_idx as usize;
                if add_src < n {
                    apply_grant(
                        bone,
                        i,
                        add_src,
                        pmx_bones,
                        &world_mats,
                        &base_rots,
                        &base_movs,
                        &grant_rots,
                        &grant_movs,
                        &mut local_rots,
                        &mut local_movs,
                        false, // FK フェーズ
                    );
                }
            }
        }

        // 付与回転量/付与移動量を保存（IKリンク回転を含まないこの時点の値）
        grant_rots[i] = local_rots[i];
        grant_movs[i] = local_movs[i];

        // ワールド行列を更新
        world_mats[i] = calc_world_mat(pmx_bones, &world_mats, &local_rots, &local_movs, i, n, extern_parent_mats);
    }

    // ── IK 計算（ik_bone_indices 順）──
    for (j, &ik_bone_idx) in ik_bone_indices.iter().enumerate() {
        if j < ik_enabled.len() && !ik_enabled[j] {
            continue;
        }
        if ik_bone_idx < 0 || (ik_bone_idx as usize) >= n {
            continue;
        }
        let ik_idx = ik_bone_idx as usize;
        if pmx_bones[ik_idx].ik.is_none() {
            continue;
        }
        solve_ccd_ik(
            pmx_bones,
            &mut world_mats,
            &mut local_rots,
            &mut local_movs,
            &base_rots,
            &base_movs,
            ik_idx,
            &sorted_indices,
            extern_parent_mats,
        );
    }

    // ワールド行列から位置と回転を取り出す
    (0..n)
        .map(|i| {
            let mat = world_mats[i];
            let translation = mat.col(3).truncate();
            let rotation = Quat::from_mat4(&mat).normalize();
            (translation, rotation)
        })
        .collect()
}

// ─────────────────────────────────────────────
// 付与処理ヘルパー
// ─────────────────────────────────────────────

/// 1ボーン分の付与処理（ローカル付与/通常付与を統合）
///
/// - FK フェーズ (`is_recompute=false`): 付与元は grant_rots/base_rots を参照
/// - IK 後の再計算 (`is_recompute=true`): 付与元は local_rots を参照（IK回転量も付与）
#[allow(clippy::too_many_arguments)]
fn apply_grant(
    bone: &pmx::PmxBone,
    i: usize,
    add_src: usize,
    pmx_bones: &[pmx::PmxBone],
    world_mats: &[Mat4],
    base_rots: &[Quat],
    base_movs: &[Vec3],
    grant_rots: &[Quat],
    grant_movs: &[Vec3],
    local_rots: &mut Vec<Quat>,
    local_movs: &mut Vec<Vec3>,
    is_recompute: bool,
) {
    let ratio = bone.add_ratio;

    if bone.is_local_add {
        // ━━ ローカル付与（仕様 bit 0x0080）━━
        let local_mat_src = {
            let parent_of_src = pmx_bones[add_src].parent_index;
            if parent_of_src >= 0 && (parent_of_src as usize) < pmx_bones.len() {
                world_mats[parent_of_src as usize].inverse() * world_mats[add_src]
            } else {
                world_mats[add_src]
            }
        };

        if bone.is_add_rotation {
            let local_rot_src = Quat::from_mat4(&local_mat_src).normalize();
            let add_rot = Quat::IDENTITY.slerp(local_rot_src, ratio);
            local_rots[i] = (base_rots[i] * add_rot).normalize();
        }
        if bone.is_add_translation {
            let local_pos_src = local_mat_src.col(3).truncate();
            let init_pos_src = pmx_bones[add_src].position;
            let delta = local_pos_src - init_pos_src;
            local_movs[i] = base_movs[i] + delta * ratio;
        }
    } else {
        // ━━ 通常付与（仕様 □付与について）━━
        let add_src_bone = &pmx_bones[add_src];
        let add_src_is_add = add_src_bone.is_add_rotation || add_src_bone.is_add_translation;

        if bone.is_add_rotation {
            let src_rot = if add_src_is_add {
                if is_recompute { local_rots[add_src] } else { grant_rots[add_src] }
            } else {
                base_rots[add_src]
            };
            let add_rot = Quat::IDENTITY.slerp(src_rot, ratio);
            local_rots[i] = (base_rots[i] * add_rot).normalize();
        }
        if bone.is_add_translation {
            let src_mov = if add_src_is_add {
                if is_recompute { local_movs[add_src] } else { grant_movs[add_src] }
            } else {
                base_movs[add_src]
            };
            local_movs[i] = base_movs[i] + src_mov * ratio;
        }
    }
}

// ─────────────────────────────────────────────
// ワールド行列計算ヘルパー
// ─────────────────────────────────────────────

/// ボーン i のワールド行列を計算して返す（world_mats[i] は更新しない）
///
/// extern_parent_mats にエントリがある場合はその行列を親として使用する。
/// 外部親時の変形式:
///   world = extern_parent_mat × T(local_movs[i]) × R(local_rots[i])
/// （外部親は別モデルの座標系のため rest_offset は 0）
fn calc_world_mat(
    pmx_bones: &[pmx::PmxBone],
    world_mats: &[Mat4],
    local_rots: &[Quat],
    local_movs: &[Vec3],
    i: usize,
    n: usize,
    extern_parent_mats: &std::collections::HashMap<usize, Mat4>,
) -> Mat4 {
    // 外部親が設定されている場合: その行列を親として使用（rest_offset = 0）
    if let Some(&extern_mat) = extern_parent_mats.get(&i) {
        let local_mat = Mat4::from_rotation_translation(local_rots[i], local_movs[i]);
        return extern_mat * local_mat;
    }

    let bone = &pmx_bones[i];
    let bone_pos = bone.position;
    let parent_idx = bone.parent_index;

    let (rest_offset, parent_mat) = if parent_idx >= 0 && (parent_idx as usize) < n {
        let parent_pos = pmx_bones[parent_idx as usize].position;
        (bone_pos - parent_pos, world_mats[parent_idx as usize])
    } else {
        (bone_pos, Mat4::IDENTITY)
    };

    let local_pos = rest_offset + local_movs[i];
    let local_mat = Mat4::from_rotation_translation(local_rots[i], local_pos);
    parent_mat * local_mat
}

// ─────────────────────────────────────────────
// CCD-IK ソルバー
// ─────────────────────────────────────────────

/// CCD-IK ソルバー
fn solve_ccd_ik(
    bones: &[pmx::PmxBone],
    world_mats: &mut Vec<Mat4>,
    local_rots: &mut Vec<Quat>,
    local_movs: &mut Vec<Vec3>,
    base_rots: &[Quat],
    base_movs: &[Vec3],
    ik_idx: usize,
    sorted_indices: &[usize],
    extern_parent_mats: &std::collections::HashMap<usize, Mat4>,
) {
    let ik = bones[ik_idx].ik.as_ref().unwrap();
    let target_raw = ik.target_bone_index;
    if target_raw < 0 || (target_raw as usize) >= bones.len() {
        return;
    }
    let target_idx = target_raw as usize;

    for _ in 0..ik.loop_count {
        for link in &ik.links {
            let link_raw = link.bone_index;
            if link_raw < 0 || (link_raw as usize) >= bones.len() {
                continue;
            }
            let link_idx = link_raw as usize;

            let goal_world = world_mats[ik_idx].col(3).truncate();
            let effector_world = world_mats[target_idx].col(3).truncate();

            let inv_link = world_mats[link_idx].inverse();
            let local_effector = inv_link.transform_point3(effector_world);
            let local_goal = inv_link.transform_point3(goal_world);

            if local_effector.length_squared() < 1e-10 || local_goal.length_squared() < 1e-10 {
                continue;
            }

            let from = local_effector.normalize();
            let to = local_goal.normalize();

            if from.dot(to) > 0.9999 {
                continue;
            }

            let rot = Quat::from_rotation_arc(from, to);
            let rot = clamp_quat_angle(rot, ik.limit_angle);

            local_rots[link_idx] = (rot * local_rots[link_idx]).normalize();

            if link.angle_min.is_some() {
                local_rots[link_idx] = apply_ik_angle_limit(local_rots[link_idx], link);
            }

            recompute_world_mats_from(
                bones, world_mats, local_rots, local_movs, base_rots, base_movs,
                link_idx, sorted_indices, extern_parent_mats,
            );
        }
    }
}

/// クォータニオンの回転角度を max_angle にクランプする
fn clamp_quat_angle(q: Quat, max_angle: f32) -> Quat {
    if max_angle <= 0.0 {
        return Quat::IDENTITY;
    }
    let (axis, angle) = q.to_axis_angle();
    if angle <= max_angle { q } else { Quat::from_axis_angle(axis, max_angle) }
}

/// IK リンクボーン固有の角度制限を適用する（XYZ オイラー角クランプ）
fn apply_ik_angle_limit(q: Quat, link: &pmx::IkLink) -> Quat {
    let min = link.angle_min.unwrap_or([-std::f32::consts::PI; 3]);
    let max = link.angle_max.unwrap_or([std::f32::consts::PI; 3]);

    let (x, y, z) = q.to_euler(EulerRot::XYZ);
    Quat::from_euler(EulerRot::XYZ,
        x.clamp(min[0], max[0]),
        y.clamp(min[1], max[1]),
        z.clamp(min[2], max[2]),
    ).normalize()
}

// ─────────────────────────────────────────────
// IK 後のワールド行列再計算
// ─────────────────────────────────────────────

fn recompute_world_mats_from(
    bones: &[pmx::PmxBone],
    world_mats: &mut Vec<Mat4>,
    local_rots: &mut Vec<Quat>,
    local_movs: &mut Vec<Vec3>,
    base_rots: &[Quat],
    base_movs: &[Vec3],
    start_bone: usize,
    sorted_indices: &[usize],
    extern_parent_mats: &std::collections::HashMap<usize, Mat4>,
) {
    let start_order = bones[start_bone].transform_order;
    let start_pos = sorted_indices
        .iter()
        .position(|&i| i == start_bone)
        .unwrap_or(0);
    let n = bones.len();

    for (pos, &i) in sorted_indices.iter().enumerate() {
        if bones[i].transform_order < start_order
            || (bones[i].transform_order == start_order && pos < start_pos)
        {
            continue;
        }

        let bone = &bones[i];

        if bone.is_add_rotation || bone.is_add_translation {
            if let Some(add_src_idx) = bone.add_bone_index {
                let add_src = add_src_idx as usize;
                if add_src < n {
                    apply_grant(
                        bone, i, add_src, bones, world_mats,
                        base_rots, base_movs,
                        &[], &[], // grant_rots/movs（is_recompute=true のため不参照）
                        local_rots, local_movs,
                        true,
                    );
                }
            }
        }

        world_mats[i] = calc_world_mat(bones, world_mats, local_rots, local_movs, i, n, extern_parent_mats);
    }
}
