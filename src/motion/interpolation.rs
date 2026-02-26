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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::pmm::{ExternParentEntry, InterpolationCurve, PmmBone, PmmModel, PmmData};

    // ────── テストヘルパー ──────

    fn linear_ic() -> InterpolationCurve {
        InterpolationCurve { x1: 20, y1: 20, x2: 107, y2: 107 }
    }

    fn nonlinear_ic() -> InterpolationCurve {
        InterpolationCurve { x1: 20, y1: 80, x2: 107, y2: 107 }
    }

    fn make_bone_frame_at(frame: i32, x: f32, y: f32, z: f32) -> BoneFrame {
        BoneFrame {
            frame,
            interp_x: linear_ic(),
            interp_y: linear_ic(),
            interp_z: linear_ic(),
            interp_rot: linear_ic(),
            movement: Vec3::new(x, y, z),
            rotation: Quat::IDENTITY,
            physics_enabled: true,
        }
    }

    fn make_pmm_bone(name: &str, frames: Vec<BoneFrame>) -> PmmBone {
        PmmBone { name: name.to_string(), frames }
    }

    fn make_pmm_model(bones: Vec<PmmBone>) -> PmmModel {
        PmmModel {
            name: "TestModel".to_string(),
            model_id: 0,
            render_order: 0,
            path: "".to_string(),
            bones,
            ik_bone_indices: vec![],
            initial_ik_state: vec![],
            config_frames: vec![],
            parentable_bone_indices: vec![],
            initial_extern_parents: vec![],
        }
    }

    fn make_pmm_data(models: Vec<PmmModel>) -> PmmData {
        PmmData {
            output_width: 1920,
            output_height: 1080,
            models,
            cameras: vec![],
        }
    }

    // ────── lerp ──────

    #[test]
    fn test_lerp_endpoints_and_midpoint() {
        assert!((lerp(0.0, 10.0, 0.0) - 0.0).abs() < 1e-6);
        assert!((lerp(0.0, 10.0, 1.0) - 10.0).abs() < 1e-6);
        assert!((lerp(0.0, 10.0, 0.5) - 5.0).abs() < 1e-6);
    }

    // ────── interp_value ──────

    #[test]
    fn test_interp_value_linear() {
        let ic = linear_ic();
        let result = interp_value(0.0, 10.0, 0.5, &ic);
        assert!((result - 5.0).abs() < 1e-5, "expected 5.0, got {}", result);
    }

    #[test]
    fn test_interp_value_nonlinear() {
        let ic = nonlinear_ic(); // x1=20, y1=80 → not linear
        let linear_mid = lerp(0.0, 10.0, 0.5);
        let nonlinear_mid = interp_value(0.0, 10.0, 0.5, &ic);
        assert!(
            (linear_mid - nonlinear_mid).abs() > 1.0,
            "linear={}, nonlinear={}", linear_mid, nonlinear_mid
        );
    }

    // ────── frames_approx_equal ──────

    #[test]
    fn test_frames_approx_equal_same() {
        assert!(frames_approx_equal((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)));
    }

    #[test]
    fn test_frames_approx_equal_diff() {
        assert!(!frames_approx_equal((1.0, 2.0, 3.0), (1.01, 2.0, 3.0)));
    }

    #[test]
    fn test_frames_approx_equal_epsilon() {
        // 差 5e-7 < EPS(1e-6) → true
        assert!(frames_approx_equal((1.0, 2.0, 3.0), (1.0 + 5e-7, 2.0, 3.0)));
    }

    // ────── needs_bezier ──────

    #[test]
    fn test_needs_bezier_linear() {
        let fa = make_bone_frame_at(0, 0.0, 0.0, 0.0);
        let fb = make_bone_frame_at(10, 1.0, 0.0, 0.0);
        assert!(!needs_bezier(&fa, &fb));
    }

    #[test]
    fn test_needs_bezier_nonlinear_x() {
        let fa = make_bone_frame_at(0, 0.0, 0.0, 0.0);
        let mut fb = make_bone_frame_at(10, 1.0, 0.0, 0.0);
        fb.interp_x = nonlinear_ic();
        assert!(needs_bezier(&fa, &fb));
    }

    // ────── interp_bone_at_frame ──────

    #[test]
    fn test_interp_bone_empty() {
        let bone = make_pmm_bone("empty", vec![]);
        let (mov, rot) = interp_bone_at_frame(&bone, 5);
        assert_eq!(mov, Vec3::ZERO);
        assert_eq!(rot, Quat::IDENTITY);
    }

    #[test]
    fn test_interp_bone_before_start() {
        let bone = make_pmm_bone("b", vec![make_bone_frame_at(10, 1.0, 0.0, 0.0)]);
        let (mov, _) = interp_bone_at_frame(&bone, 0);
        assert!((mov.x - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_interp_bone_after_end() {
        let bone = make_pmm_bone("b", vec![make_bone_frame_at(10, 5.0, 0.0, 0.0)]);
        let (mov, _) = interp_bone_at_frame(&bone, 100);
        assert!((mov.x - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_interp_bone_midpoint_linear() {
        let bone = make_pmm_bone("b", vec![
            make_bone_frame_at(0, 0.0, 0.0, 0.0),
            make_bone_frame_at(10, 10.0, 0.0, 0.0),
        ]);
        let (mov, _) = interp_bone_at_frame(&bone, 5);
        assert!((mov.x - 5.0).abs() < 1e-5, "expected 5.0, got {}", mov.x);
    }

    // ────── get_ik_enabled_at ──────

    #[test]
    fn test_get_ik_enabled_no_config() {
        let model = PmmModel {
            initial_ik_state: vec![true, false],
            config_frames: vec![],
            ..make_pmm_model(vec![])
        };
        let result = get_ik_enabled_at(&model, 100);
        assert_eq!(result, vec![true, false]);
    }

    #[test]
    fn test_get_ik_enabled_latest_before_frame() {
        let model = PmmModel {
            initial_ik_state: vec![true, true],
            config_frames: vec![
                ConfigFrame { frame: 10, ik_enabled: vec![false, true], extern_parents: vec![] },
                ConfigFrame { frame: 20, ik_enabled: vec![false, false], extern_parents: vec![] },
            ],
            ..make_pmm_model(vec![])
        };
        // frame 15 → latest config before 15 is frame 10
        let r = get_ik_enabled_at(&model, 15);
        assert_eq!(r, vec![false, true]);
        // frame 5 → no config before 5 → initial
        let r2 = get_ik_enabled_at(&model, 5);
        assert_eq!(r2, vec![true, true]);
    }

    // ────── get_extern_parents_at ──────

    #[test]
    fn test_get_extern_parents_at_initial() {
        let ep = ExternParentEntry { model_index: -1, bone_index: -1 };
        let model = PmmModel {
            initial_extern_parents: vec![ep.clone()],
            config_frames: vec![],
            ..make_pmm_model(vec![])
        };
        let result = get_extern_parents_at(&model, 0);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].model_index, -1);
    }

    // ────── build_frames (no FK) ──────

    #[test]
    fn test_build_frames_no_fk_linear_keyframes() {
        let bone = make_pmm_bone("Center", vec![
            make_bone_frame_at(0, 0.0, 0.0, 0.0),
            make_bone_frame_at(10, 10.0, 0.0, 0.0),
        ]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);

        let result = build_frames(&pmm_data, 0, &[None], 0, 30, false);
        assert_eq!(result.len(), 2, "expected 2 keyframes");
        assert_eq!(result[0].frame, 0);
        assert_eq!(result[1].frame, 10);
        assert!((result[1].world_x - 10.0).abs() < 1e-5);
    }

    #[test]
    fn test_build_frames_no_fk_all_frames() {
        let bone = make_pmm_bone("Center", vec![
            make_bone_frame_at(0, 0.0, 0.0, 0.0),
            make_bone_frame_at(10, 10.0, 0.0, 0.0),
        ]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);

        let result = build_frames(&pmm_data, 0, &[None], 0, 30, true);
        // 全フレームモード: frames 0..=10 で全て異なる値 → 11要素
        assert_eq!(result.len(), 11, "expected 11 frames");
        assert_eq!(result[0].frame, 0);
        assert_eq!(result[10].frame, 10);
    }

    #[test]
    fn test_build_frames_no_fk_value_skip() {
        // 値が変化しないフレームはスキップされる
        let bone = make_pmm_bone("Static", vec![
            make_bone_frame_at(0, 0.0, 0.0, 0.0),
            make_bone_frame_at(10, 0.0, 0.0, 0.0), // 同じ値
        ]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);

        let result = build_frames(&pmm_data, 0, &[None], 0, 30, true);
        // 全フレーム(0-10)が全て(0,0,0) → スキップされて1要素のみ
        assert_eq!(result.len(), 1, "expected 1 frame (rest skipped), got {}", result.len());
    }

    #[test]
    fn test_build_frames_empty_bone() {
        let bone = make_pmm_bone("Empty", vec![]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);

        let result = build_frames(&pmm_data, 0, &[None], 0, 30, false);
        assert!(result.is_empty());
    }

    // ────── build_frames (with PMX/FK) ──────

    #[test]
    fn test_build_frames_with_fk_parent_child() {
        // PMX: parent(0,0,0) → child(0,10,0)
        // PMM: parent に Y=5 移動, child に 0 移動
        // child world Y = 5 + 10 = 15
        let pmx_bones = vec![
            PmxBone {
                name: "parent".to_string(),
                position: Vec3::new(0.0, 0.0, 0.0),
                parent_index: -1,
                transform_order: 0,
                ik: None, add_bone_index: None, add_ratio: 0.0,
                is_local_add: false, is_add_rotation: false, is_add_translation: false,
            },
            PmxBone {
                name: "child".to_string(),
                position: Vec3::new(0.0, 10.0, 0.0),
                parent_index: 0,
                transform_order: 0,
                ik: None, add_bone_index: None, add_ratio: 0.0,
                is_local_add: false, is_add_rotation: false, is_add_translation: false,
            },
        ];
        let parent_bone = make_pmm_bone("parent", vec![make_bone_frame_at(0, 0.0, 5.0, 0.0)]);
        let child_bone  = make_pmm_bone("child",  vec![make_bone_frame_at(0, 0.0, 0.0, 0.0)]);
        let model = make_pmm_model(vec![parent_bone, child_bone]);
        let pmm_data = make_pmm_data(vec![model]);
        // target_bone_idx=1 は PMM の child ボーン
        let result = build_frames(&pmm_data, 0, &[Some(pmx_bones)], 1, 30, false);
        assert_eq!(result.len(), 1);
        assert!((result[0].world_y - 15.0).abs() < 1e-4, "expected y=15.0, got {}", result[0].world_y);
    }

    #[test]
    fn test_build_frames_with_fk_all_frames() {
        // PMX FK パス + all_frames=true
        let pmx_bones = vec![
            PmxBone {
                name: "Center".to_string(),
                position: Vec3::ZERO,
                parent_index: -1,
                transform_order: 0,
                ik: None, add_bone_index: None, add_ratio: 0.0,
                is_local_add: false, is_add_rotation: false, is_add_translation: false,
            },
        ];
        let bone = make_pmm_bone("Center", vec![
            make_bone_frame_at(0, 0.0, 0.0, 0.0),
            make_bone_frame_at(5, 5.0, 0.0, 0.0),
        ]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);
        let result = build_frames(&pmm_data, 0, &[Some(pmx_bones)], 0, 30, true);
        // フレーム 0..=5 で全て異なる値 → 6フレーム
        assert_eq!(result.len(), 6, "expected 6 frames, got {}", result.len());
    }

    // ────── compute_model_world_transforms_at ──────

    #[test]
    fn test_compute_model_world_transforms_at_single_bone() {
        let pmx_bones = vec![
            PmxBone {
                name: "Center".to_string(),
                position: Vec3::ZERO,
                parent_index: -1,
                transform_order: 0,
                ik: None, add_bone_index: None, add_ratio: 0.0,
                is_local_add: false, is_add_rotation: false, is_add_translation: false,
            },
        ];
        let bone = make_pmm_bone("Center", vec![make_bone_frame_at(0, 1.0, 2.0, 3.0)]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);
        let result = compute_model_world_transforms_at(&pmm_data, 0, &[Some(pmx_bones)], 0);
        assert!(result.is_some());
        let transforms = result.unwrap();
        assert_eq!(transforms.len(), 1);
        let (pos, _) = transforms[0];
        assert!((pos.x - 1.0).abs() < 1e-5);
        assert!((pos.y - 2.0).abs() < 1e-5);
        assert!((pos.z - 3.0).abs() < 1e-5);
    }

    #[test]
    fn test_compute_model_world_transforms_at_no_pmx() {
        // PMXなし → None を返す
        let bone = make_pmm_bone("Center", vec![make_bone_frame_at(0, 1.0, 0.0, 0.0)]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);
        let result = compute_model_world_transforms_at(&pmm_data, 0, &[None], 0);
        assert!(result.is_none());
    }

    // ────── get_bone_world_pos_at ──────

    #[test]
    fn test_get_bone_world_pos_at() {
        let pmx_bones = vec![
            PmxBone {
                name: "Center".to_string(),
                position: Vec3::ZERO,
                parent_index: -1,
                transform_order: 0,
                ik: None, add_bone_index: None, add_ratio: 0.0,
                is_local_add: false, is_add_rotation: false, is_add_translation: false,
            },
        ];
        let bone = make_pmm_bone("Center", vec![make_bone_frame_at(0, 5.0, 3.0, 0.0)]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);
        let pos = get_bone_world_pos_at(&pmm_data, 0, &[Some(pmx_bones)], 0, 0);
        assert!(pos.is_some());
        let p = pos.unwrap();
        assert!((p.x - 5.0).abs() < 1e-5, "expected x=5.0, got {}", p.x);
        assert!((p.y - 3.0).abs() < 1e-5, "expected y=3.0, got {}", p.y);
    }

    #[test]
    fn test_get_bone_world_pos_at_out_of_range() {
        let pmx_bones = vec![
            PmxBone {
                name: "Center".to_string(),
                position: Vec3::ZERO,
                parent_index: -1,
                transform_order: 0,
                ik: None, add_bone_index: None, add_ratio: 0.0,
                is_local_add: false, is_add_rotation: false, is_add_translation: false,
            },
        ];
        let bone = make_pmm_bone("Center", vec![make_bone_frame_at(0, 0.0, 0.0, 0.0)]);
        let model = make_pmm_model(vec![bone]);
        let pmm_data = make_pmm_data(vec![model]);
        // bone_pmx_idx=99 は範囲外 → None
        let pos = get_bone_world_pos_at(&pmm_data, 0, &[Some(pmx_bones)], 99, 0);
        assert!(pos.is_none());
    }
}
