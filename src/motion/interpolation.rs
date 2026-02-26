// ベジェ補間・フレーム構築モジュール
// PMMキーフレームをベジェ補間し、FK/IK 計算を経て全フレームの位置データを構築する。

use crate::format::pmm::{BoneFrame, ExternParentEntry, InterpolationCurve, PmmBone, PmmModel,
                 PmmData, ConfigFrame};
use crate::format::pmx::PmxBone;
use crate::jsx::bone::FrameData;
use super::transform;
use super::bezier::BezierCurve;
use std::collections::HashMap;
use glam::{Mat4, Quat, Vec3};

// ─────────────────────────────────────────────
// ベジェ補間ヘルパー
// ─────────────────────────────────────────────

/// InterpolationCurve（0-127スケール）を BezierCurve（0-1正規化）に変換
#[inline]
fn to_bezier(c: &InterpolationCurve) -> BezierCurve {
    const N: f64 = 127.0;
    BezierCurve::new(c.x1 as f64 / N, c.y1 as f64 / N, c.x2 as f64 / N, c.y2 as f64 / N)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// ベジェ補間（各軸独立）
fn interp_value(a: f32, b: f32, t: f32, curve: &InterpolationCurve) -> f32 {
    if curve.is_linear() {
        lerp(a, b, t)
    } else {
        lerp(a, b, to_bezier(curve).evaluate(t as f64) as f32)
    }
}

/// キーフレーム間がベジェ補間を必要とするか
fn needs_bezier(_fa: &BoneFrame, fb: &BoneFrame) -> bool {
    !fb.interp_x.is_linear()
        || !fb.interp_y.is_linear()
        || !fb.interp_z.is_linear()
        || !fb.interp_rot.is_linear()
}

// ─────────────────────────────────────────────
// 単一ボーン補間ヘルパー
// ─────────────────────────────────────────────

/// 指定フレーム番号でのボーンの移動量と回転を補間して返す
fn interp_bone_at_frame(bone: &PmmBone, frame: i32) -> (Vec3, Quat) {
    let frames = &bone.frames;
    if frames.is_empty() {
        return (Vec3::ZERO, Quat::IDENTITY);
    }

    if frame <= frames[0].frame {
        return (frames[0].movement, frames[0].rotation);
    }

    let last = frames.last().unwrap();
    if frame >= last.frame {
        return (last.movement, last.rotation);
    }

    let idx = frames.partition_point(|f| f.frame <= frame);
    let fa = &frames[idx - 1];
    let fb = &frames[idx];

    if fa.frame == frame {
        return (fa.movement, fa.rotation);
    }

    let frame_diff = fb.frame - fa.frame;
    let t = (frame - fa.frame) as f32 / frame_diff as f32;

    let mx = interp_value(fa.movement.x, fb.movement.x, t, &fb.interp_x);
    let my = interp_value(fa.movement.y, fb.movement.y, t, &fb.interp_y);
    let mz = interp_value(fa.movement.z, fb.movement.z, t, &fb.interp_z);
    let rot_t = to_bezier(&fb.interp_rot).evaluate(t as f64) as f32;
    let rot = fa.rotation.slerp(fb.rotation, rot_t);

    (Vec3::new(mx, my, mz), rot)
}

/// 指定フレームの IK 有効状態を取得する
fn get_ik_enabled_at(pmm_model: &PmmModel, frame: i32) -> Vec<bool> {
    let applicable = pmm_model
        .config_frames
        .iter()
        .filter(|cf| cf.frame <= frame)
        .max_by_key(|cf| cf.frame);

    match applicable {
        Some(cf) => cf.ik_enabled.clone(),
        None => pmm_model.initial_ik_state.clone(),
    }
}

/// 指定フレームの外部親情報を取得する
fn get_extern_parents_at(pmm_model: &PmmModel, frame: i32) -> &[ExternParentEntry] {
    let applicable = pmm_model
        .config_frames
        .iter()
        .filter(|cf| cf.frame <= frame)
        .max_by_key(|cf| cf.frame);

    match applicable {
        Some(cf) => &cf.extern_parents,
        None => &pmm_model.initial_extern_parents,
    }
}

/// モデル `model_idx` の全ボーンのワールド変換を計算して返す（外部親なし）
pub(crate) fn compute_model_world_transforms_at(
    pmm_data: &PmmData,
    model_idx: usize,
    all_pmx_bones: &[Option<Vec<PmxBone>>],
    frame: i32,
) -> Option<Vec<(Vec3, Quat)>> {
    let pmm_model = pmm_data.models.get(model_idx)?;
    let pmx_bones = all_pmx_bones.get(model_idx)?.as_ref()?;

    let pmx_to_pmm: Vec<Option<usize>> = pmx_bones
        .iter()
        .map(|pb| pmm_model.bones.iter().position(|mb| mb.name == pb.name))
        .collect();

    let mut movements: Vec<Vec3> = vec![Vec3::ZERO; pmx_bones.len()];
    let mut rotations: Vec<Quat> = vec![Quat::IDENTITY; pmx_bones.len()];

    for pmx_idx in 0..pmx_bones.len() {
        if let Some(pmm_idx) = pmx_to_pmm[pmx_idx] {
            let (mov, rot) = interp_bone_at_frame(&pmm_model.bones[pmm_idx], frame);
            movements[pmx_idx] = mov;
            rotations[pmx_idx] = rot;
        }
    }

    let ik_enabled = get_ik_enabled_at(pmm_model, frame);
    Some(transform::compute_world_transforms(
        pmx_bones,
        &movements,
        &rotations,
        &ik_enabled,
        &pmm_model.ik_bone_indices,
        &HashMap::new(), // 外部親の外部親は再帰防止のため無視
    ))
}

/// 指定フレームの指定ボーンのワールド位置を返す（外部親なし）
///
/// カメラ追従ボーン位置の計算用。外部親は再帰防止のため無視される。
pub(crate) fn get_bone_world_pos_at(
    pmm_data: &PmmData,
    model_arr_idx: usize,
    all_pmx_bones: &[Option<Vec<PmxBone>>],
    bone_pmx_idx: usize,
    frame: i32,
) -> Option<glam::Vec3> {
    let transforms = compute_model_world_transforms_at(pmm_data, model_arr_idx, all_pmx_bones, frame)?;
    transforms.get(bone_pmx_idx).map(|(pos, _)| *pos)
}

/// 対象モデルの外部親マップを構築する
///
/// 戻り値: PMXボーンインデックス → 外部親ワールド行列
fn build_extern_parent_mats(
    pmm_data: &PmmData,
    pmm_model_idx: usize,
    pmm_model: &PmmModel,
    pmx_bones: &[PmxBone],
    all_pmx_bones: &[Option<Vec<PmxBone>>],
    frame: i32,
) -> HashMap<usize, Mat4> {
    let model_id_to_arr: HashMap<u8, usize> = pmm_data
        .models
        .iter()
        .enumerate()
        .map(|(arr_idx, m)| (m.model_id, arr_idx))
        .collect();

    let mut result = HashMap::new();
    let extern_parents = get_extern_parents_at(pmm_model, frame);

    let mut cache: HashMap<usize, Vec<(Vec3, Quat)>> = HashMap::new();

    for (k, ep) in extern_parents.iter().enumerate() {
        if ep.model_index < 0 || ep.bone_index < 0 {
            continue;
        }
        if k >= pmm_model.parentable_bone_indices.len() {
            continue;
        }

        let ref_model_id = ep.model_index as u8;
        let ref_arr_idx = match model_id_to_arr.get(&ref_model_id) {
            Some(&idx) => idx,
            None => continue,
        };

        let ref_bone_idx = ep.bone_index as usize;
        let target_pmx_bone_idx = pmm_model.parentable_bone_indices[k] as usize;

        if ref_arr_idx == pmm_model_idx || target_pmx_bone_idx >= pmx_bones.len() {
            continue;
        }

        if !cache.contains_key(&ref_arr_idx) {
            if let Some(transforms) =
                compute_model_world_transforms_at(pmm_data, ref_arr_idx, all_pmx_bones, frame)
            {
                cache.insert(ref_arr_idx, transforms);
            }
        }

        if let Some(ref_transforms) = cache.get(&ref_arr_idx) {
            if ref_bone_idx < ref_transforms.len() {
                let (pos, rot) = ref_transforms[ref_bone_idx];
                result.insert(target_pmx_bone_idx, Mat4::from_rotation_translation(rot, pos));
            }
        }
    }

    result
}

// ─────────────────────────────────────────────
// メイン公開関数
// ─────────────────────────────────────────────

/// ワールド座標が実質的に等しいか判定（浮動小数点誤差を許容）
fn frames_approx_equal(a: (f32, f32, f32), b: (f32, f32, f32)) -> bool {
    const EPS: f32 = 1e-6;
    (a.0 - b.0).abs() < EPS
        && (a.1 - b.1).abs() < EPS
        && (a.2 - b.2).abs() < EPS
}

/// 全ボーンのフレームデータを計算して対象ボーンの出力データを返す
pub fn build_frames(
    pmm_data: &PmmData,
    pmm_model_idx: usize,
    all_pmx_bones: &[Option<Vec<PmxBone>>],
    target_bone_idx: usize,
    _fps: u32,
    all_frames: bool,
) -> Vec<FrameData> {
    let pmm_model = &pmm_data.models[pmm_model_idx];
    let target_bone = &pmm_model.bones[target_bone_idx];
    let frames = &target_bone.frames;

    if frames.is_empty() {
        return Vec::new();
    }

    let pmx_bones_opt = all_pmx_bones.get(pmm_model_idx).and_then(|m| m.as_ref());
    let use_fk = pmx_bones_opt.map(|b| !b.is_empty()).unwrap_or(false);

    if !use_fk {
        return build_frames_no_fk(target_bone, all_frames);
    }

    let pmx_bones = pmx_bones_opt.unwrap();

    let pmx_to_pmm: Vec<Option<usize>> = pmx_bones
        .iter()
        .map(|pb| pmm_model.bones.iter().position(|mb| mb.name == pb.name))
        .collect();

    let target_pmx_idx = pmx_bones.iter().position(|b| b.name == target_bone.name);

    let compute_world_pos = |frame: i32| -> (f32, f32, f32) {
        let mut movements: Vec<Vec3> = vec![Vec3::ZERO; pmx_bones.len()];
        let mut rotations: Vec<Quat> = vec![Quat::IDENTITY; pmx_bones.len()];

        for pmx_idx in 0..pmx_bones.len() {
            if let Some(pmm_idx) = pmx_to_pmm[pmx_idx] {
                let (mov, rot) = interp_bone_at_frame(&pmm_model.bones[pmm_idx], frame);
                movements[pmx_idx] = mov;
                rotations[pmx_idx] = rot;
            }
        }

        let ik_enabled = get_ik_enabled_at(pmm_model, frame);

        let extern_parent_mats = build_extern_parent_mats(
            pmm_data, pmm_model_idx, pmm_model, pmx_bones, all_pmx_bones, frame,
        );

        let world_transforms = transform::compute_world_transforms(
            pmx_bones, &movements, &rotations, &ik_enabled,
            &pmm_model.ik_bone_indices, &extern_parent_mats,
        );

        if let Some(pmx_idx) = target_pmx_idx {
            let (pos, _rot) = world_transforms[pmx_idx];
            (pos.x, pos.y, pos.z)
        } else {
            let (mov, _rot) = interp_bone_at_frame(target_bone, frame);
            (mov.x, mov.y, mov.z)
        }
    };

    // ─── 全フレームモード ───
    if all_frames {
        let mut first_frame = pmm_model
            .bones
            .iter()
            .filter_map(|b| b.frames.first().map(|f| f.frame))
            .min()
            .unwrap_or(frames[0].frame);
        let mut last_frame = pmm_model
            .bones
            .iter()
            .filter_map(|b| b.frames.last().map(|f| f.frame))
            .max()
            .unwrap_or(frames.last().unwrap().frame);

        {
            let model_id_to_arr: HashMap<u8, usize> = pmm_data
                .models
                .iter()
                .enumerate()
                .map(|(arr_idx, m)| (m.model_id, arr_idx))
                .collect();

            let mut ref_arr_indices: std::collections::HashSet<usize> =
                std::collections::HashSet::new();

            let all_ep_lists = std::iter::once(pmm_model.initial_extern_parents.as_slice())
                .chain(pmm_model.config_frames.iter().map(|cf| cf.extern_parents.as_slice()));

            for ep_list in all_ep_lists {
                for ep in ep_list {
                    if ep.model_index < 0 { continue; }
                    let ref_id = ep.model_index as u8;
                    if let Some(&arr_idx) = model_id_to_arr.get(&ref_id) {
                        if arr_idx != pmm_model_idx {
                            ref_arr_indices.insert(arr_idx);
                        }
                    }
                }
            }

            for arr_idx in ref_arr_indices {
                if let Some(ref_model) = pmm_data.models.get(arr_idx) {
                    if let Some(ref_first) = ref_model.bones.iter()
                        .filter_map(|b| b.frames.first().map(|f| f.frame)).min()
                    {
                        first_frame = first_frame.min(ref_first);
                    }
                    if let Some(ref_last) = ref_model.bones.iter()
                        .filter_map(|b| b.frames.last().map(|f| f.frame)).max()
                    {
                        last_frame = last_frame.max(ref_last);
                    }
                }
            }
        }

        let mut result = Vec::new();
        let mut prev_key: Option<(f32, f32, f32)> = None;

        for frame_no in first_frame..=last_frame {
            let (wx, wy, wz) = compute_world_pos(frame_no);
            let cur_key = (wx, wy, wz);

            if let Some(p) = prev_key {
                if frames_approx_equal(p, cur_key) {
                    prev_key = Some(cur_key);
                    continue;
                }
            }

            result.push(FrameData { frame: frame_no, world_x: wx, world_y: wy, world_z: wz });
            prev_key = Some(cur_key);
        }
        return result;
    }

    // ─── キーフレームのみモード ───
    let mut result = Vec::new();

    let (wx, wy, wz) = compute_world_pos(frames[0].frame);
    result.push(FrameData { frame: frames[0].frame, world_x: wx, world_y: wy, world_z: wz });

    for window in frames.windows(2) {
        let fa = &window[0];
        let fb = &window[1];
        let frame_diff = fb.frame - fa.frame;

        if frame_diff <= 0 { continue; }

        if needs_bezier(fa, fb) {
            for f in 1..=frame_diff {
                let frame = fa.frame + f;
                let (wx, wy, wz) = compute_world_pos(frame);
                result.push(FrameData { frame, world_x: wx, world_y: wy, world_z: wz });
            }
        } else {
            let (wx, wy, wz) = compute_world_pos(fb.frame);
            result.push(FrameData { frame: fb.frame, world_x: wx, world_y: wy, world_z: wz });
        }
    }

    result
}

/// FK/IK なし時のフレーム構築（PMX が利用できない場合）
fn build_frames_no_fk(target_bone: &PmmBone, all_frames: bool) -> Vec<FrameData> {
    let frames = &target_bone.frames;
    let mut result = Vec::new();

    if all_frames {
        let first_frame = frames[0].frame;
        let last_frame = frames.last().unwrap().frame;
        let mut prev_key: Option<(f32, f32, f32)> = None;

        for frame_no in first_frame..=last_frame {
            let (mov, _) = interp_bone_at_frame(target_bone, frame_no);
            let cur_key = (mov.x, mov.y, mov.z);

            if let Some(p) = prev_key {
                if frames_approx_equal(p, cur_key) {
                    prev_key = Some(cur_key);
                    continue;
                }
            }

            result.push(FrameData { frame: frame_no, world_x: mov.x, world_y: mov.y, world_z: mov.z });
            prev_key = Some(cur_key);
        }
        return result;
    }

    let f0 = &frames[0];
    result.push(FrameData { frame: f0.frame, world_x: f0.movement.x, world_y: f0.movement.y, world_z: f0.movement.z });

    for window in frames.windows(2) {
        let fa = &window[0];
        let fb = &window[1];
        let frame_diff = fb.frame - fa.frame;

        if frame_diff <= 0 { continue; }

        if needs_bezier(fa, fb) {
            for f in 1..=frame_diff {
                let t = f as f32 / frame_diff as f32;
                let mx = interp_value(fa.movement.x, fb.movement.x, t, &fb.interp_x);
                let my = interp_value(fa.movement.y, fb.movement.y, t, &fb.interp_y);
                let mz = interp_value(fa.movement.z, fb.movement.z, t, &fb.interp_z);
                result.push(FrameData { frame: fa.frame + f, world_x: mx, world_y: my, world_z: mz });
            }
        } else {
            result.push(FrameData { frame: fb.frame, world_x: fb.movement.x, world_y: fb.movement.y, world_z: fb.movement.z });
        }
    }

    result
}

// interpolation.rs が直接参照していないが pmm からインポートした ConfigFrame を使わないと
// dead_code 警告が出るため suppress する
#[allow(dead_code)]
fn _use_config_frame(_: &ConfigFrame) {}
