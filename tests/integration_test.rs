// 統合テスト: mmd2jsx の公開 API を使用したテスト
// lib.rs で公開されている pub モジュールのみを使用する

use mmd2jsx::format::pmm::{BoneFrame, InterpolationCurve, PmmBone, PmmModel, PmmData,
                            PmmCameraFrame, PmmCameraInterp};
use mmd2jsx::format::pmx::PmxBone;
use mmd2jsx::format::vmd::VmdCamera;
use mmd2jsx::motion::bezier::BezierCurve;
use mmd2jsx::motion::interpolation::build_frames;
use mmd2jsx::motion::transform::compute_world_transforms;
use mmd2jsx::jsx::bone::{FrameData, JsxConfig, output_jsx};
use mmd2jsx::jsx::camera::{CameraJsxConfig, output_jsx_from_vmd, output_jsx_from_pmm};
use glam::{Quat, Vec3};
use std::collections::HashMap;

// ────── テストヘルパー ──────

fn linear_ic() -> InterpolationCurve {
    InterpolationCurve { x1: 20, y1: 20, x2: 107, y2: 107 }
}

fn make_frame(frame: i32, x: f32, y: f32, z: f32) -> BoneFrame {
    BoneFrame {
        frame,
        interp_x: linear_ic(), interp_y: linear_ic(),
        interp_z: linear_ic(), interp_rot: linear_ic(),
        movement: Vec3::new(x, y, z),
        rotation: Quat::IDENTITY,
        physics_enabled: true,
    }
}

fn make_pmm_data_single_bone(frames: Vec<BoneFrame>) -> PmmData {
    let bone = PmmBone { name: "Center".to_string(), frames };
    let model = PmmModel {
        name: "TestModel".to_string(),
        model_id: 0,
        render_order: 0,
        path: "".to_string(),
        bones: vec![bone],
        ik_bone_indices: vec![],
        initial_ik_state: vec![],
        config_frames: vec![],
        parentable_bone_indices: vec![],
        initial_extern_parents: vec![],
    };
    PmmData {
        output_width: 1920, output_height: 1080,
        models: vec![model], cameras: vec![],
    }
}

fn make_pmx_bone_simple(name: &str, pos: [f32; 3], parent: i32) -> PmxBone {
    PmxBone {
        name: name.to_string(),
        position: Vec3::from_array(pos),
        parent_index: parent,
        transform_order: 0,
        ik: None,
        add_bone_index: None,
        add_ratio: 0.0,
        is_local_add: false,
        is_add_rotation: false,
        is_add_translation: false,
    }
}

// ────── BezierCurve 公開 API ──────

#[test]
fn test_bezier_evaluate_linear_pub_api() {
    let b = BezierCurve::new(0.25, 0.25, 0.75, 0.75);
    assert!(b.is_linear());
    assert!((b.evaluate(0.5) - 0.5).abs() < 1e-4, "evaluate(0.5)={}", b.evaluate(0.5));
    assert!((b.evaluate(0.0) - 0.0).abs() < 1e-5);
    assert!((b.evaluate(1.0) - 1.0).abs() < 1e-5);
}

#[test]
fn test_bezier_evaluate_nonlinear_pub_api() {
    let b = BezierCurve::new(0.1, 0.9, 0.1, 0.9);
    assert!(!b.is_linear());
    // fast-start curve: evaluate(0.5) > 0.5
    assert!(b.evaluate(0.5) > 0.5);
}

// ────── InterpolationCurve 公開 API ──────

#[test]
fn test_interpolation_curve_is_linear_pub_api() {
    let ic = InterpolationCurve { x1: 20, y1: 20, x2: 107, y2: 107 };
    assert!(ic.is_linear());
    let ic2 = InterpolationCurve { x1: 10, y1: 20, x2: 107, y2: 107 };
    assert!(!ic2.is_linear());
}

// ────── build_frames (no PMX) ──────

#[test]
fn test_build_frames_no_pmx_two_keyframes() {
    let pmm_data = make_pmm_data_single_bone(vec![
        make_frame(0,  0.0, 0.0, 0.0),
        make_frame(10, 10.0, 0.0, 0.0),
    ]);
    let result = build_frames(&pmm_data, 0, &[None], 0, 30, false);
    assert_eq!(result.len(), 2, "expected 2 keyframes");
    assert_eq!(result[0].frame, 0);
    assert_eq!(result[1].frame, 10);
    assert!((result[1].world_pos.x - 10.0).abs() < 1e-5);
}

#[test]
fn test_build_frames_all_frames_mode() {
    // 全フレームモード: frames 0..=10, 各フレーム異なる値 → 11要素
    let pmm_data = make_pmm_data_single_bone(vec![
        make_frame(0,  0.0, 0.0, 0.0),
        make_frame(10, 10.0, 0.0, 0.0),
    ]);
    let result = build_frames(&pmm_data, 0, &[None], 0, 30, true);
    assert_eq!(result.len(), 11, "expected 11 frames, got {}", result.len());
    assert_eq!(result[0].frame, 0);
    assert_eq!(result[10].frame, 10);
    // 中間値の確認: frame 5 → x ≈ 5.0
    assert!((result[5].world_pos.x - 5.0).abs() < 0.1, "frame5 x={}", result[5].world_pos.x);
}

#[test]
fn test_build_frames_empty_bone() {
    let pmm_data = make_pmm_data_single_bone(vec![]);
    let result = build_frames(&pmm_data, 0, &[None], 0, 30, false);
    assert!(result.is_empty());
}

// ────── compute_world_transforms 公開 API ──────

#[test]
fn test_compute_world_transforms_pub() {
    // 親子ボーン FK 伝播: 親 Y=5 移動 → 子(rest Y=10) world Y=15
    let bones = [
        make_pmx_bone_simple("parent", [0.0, 0.0, 0.0], -1),
        make_pmx_bone_simple("child",  [0.0, 10.0, 0.0], 0),
    ];
    let movements = [Vec3::new(0.0, 5.0, 0.0), Vec3::ZERO];
    let rotations = [Quat::IDENTITY, Quat::IDENTITY];
    let result = compute_world_transforms(
        &bones, &movements, &rotations, &[], &[], &HashMap::new(),
    );
    assert_eq!(result.len(), 2);
    let (child_pos, _) = result[1];
    assert!((child_pos.y - 15.0).abs() < 1e-4, "expected y=15.0, got {}", child_pos.y);
}

// ────── output_jsx 公開 API ──────

#[test]
fn test_build_and_output_jsx_to_file() {
    let pmm_data = make_pmm_data_single_bone(vec![
        make_frame(0, 0.0, 0.0, 0.0),
        make_frame(30, 5.0, 0.0, 0.0),
    ]);
    let frames = build_frames(&pmm_data, 0, &[None], 0, 30, false);
    assert_eq!(frames.len(), 2);

    let all_frames = vec![frames];
    let config = JsxConfig {
        width: 1920, height: 1080, aspect: 1.0,
        fps: 30, scale: 20.0,
        comp_name: "TestComp".to_string(),
        bone_names: vec!["Center".to_string()],
    };

    let tmp = std::env::temp_dir().join("mmd2jsx_test_output.jsx");
    output_jsx(&all_frames, &config, Some(&tmp)).unwrap();

    let content = std::fs::read_to_string(&tmp).unwrap();
    assert!(content.contains("MikuMikuDance To After Effects (Bone)"));
    assert!(content.contains("TestComp"));
    assert!(content.contains("Center"));

    let _ = std::fs::remove_file(&tmp);
}

#[test]
fn test_output_jsx_coordinate_content() {
    // frame=0, world_x=1, world_y=2, world_z=3, scale=20, 1920x1080
    // AE X=980, AE Y=500, AE Z=60
    let all_frames = vec![vec![FrameData {
        frame: 0, world_pos: glam::Vec3::new(1.0, 2.0, 3.0),
    }]];
    let config = JsxConfig {
        width: 1920, height: 1080, aspect: 1.0,
        fps: 30, scale: 20.0,
        comp_name: "Comp".to_string(),
        bone_names: vec!["Bone".to_string()],
    };
    let tmp = std::env::temp_dir().join("mmd2jsx_test_coord.jsx");
    output_jsx(&all_frames, &config, Some(&tmp)).unwrap();
    let content = std::fs::read_to_string(&tmp).unwrap();
    assert!(content.contains("980.000000"), "AE X not found");
    assert!(content.contains("500.000000"), "AE Y not found");
    assert!(content.contains("60.000000"),  "AE Z not found");
    let _ = std::fs::remove_file(&tmp);
}

// ────── build_frames (with PMX / FK) ──────

#[test]
fn test_build_frames_with_pmx_fk() {
    // 親ボーン(Y=5移動) + 子ボーン(rest Y=10) → child world_y = 15.0
    let pmx_bones = vec![
        make_pmx_bone_simple("parent", [0.0,  0.0, 0.0], -1),
        make_pmx_bone_simple("child",  [0.0, 10.0, 0.0],  0),
    ];
    let model = PmmModel {
        name: "TestModel".to_string(),
        model_id: 0,
        render_order: 0,
        path: "".to_string(),
        bones: vec![
            PmmBone { name: "parent".to_string(), frames: vec![make_frame(0, 0.0, 5.0, 0.0)] },
            PmmBone { name: "child".to_string(),  frames: vec![make_frame(0, 0.0, 0.0, 0.0)] },
        ],
        ik_bone_indices: vec![],
        initial_ik_state: vec![],
        config_frames: vec![],
        parentable_bone_indices: vec![],
        initial_extern_parents: vec![],
    };
    let pmm_data = PmmData {
        output_width: 1920, output_height: 1080,
        models: vec![model], cameras: vec![],
    };
    // child ボーン（PMX index=1）の world 座標を build_frames 経由で取得
    let result = build_frames(&pmm_data, 0, &[Some(pmx_bones)], 1, 30, false);
    assert_eq!(result.len(), 1);
    assert!((result[0].world_pos.y - 15.0).abs() < 1e-4,
        "expected world_y=15.0, got {}", result[0].world_pos.y);
}

// ────── output_jsx_from_vmd 公開 API ──────

fn make_linear_vmd_interp() -> [u8; 24] {
    [20, 20, 107, 107,  20, 20, 107, 107,
     20, 20, 107, 107,  20, 20, 107, 107,
     20, 20, 107, 107,  20, 20, 107, 107]
}

fn make_vmd_camera(frame_no: u32, location: [f32; 3], length: f32, fov: u32) -> VmdCamera {
    VmdCamera {
        frame_no,
        length,
        location,
        rotation: [0.0, 0.0, 0.0],
        interpolation: make_linear_vmd_interp(),
        viewing_angle: fov,
        perspective: 0,
    }
}

#[test]
fn test_output_jsx_from_vmd_single_frame() {
    let config = CameraJsxConfig {
        width: 1920, height: 1080, pixel_aspect: 1.0,
        fps: 30,
        comp_name: "VmdComp".to_string(),
        camera_name: "VmdCam".to_string(),
    };
    let tmp = std::env::temp_dir().join("mmd2jsx_test_vmd_cam.jsx");
    output_jsx_from_vmd(&[make_vmd_camera(0, [0.0, 10.0, 0.0], -45.0, 30)], &config, Some(&tmp)).unwrap();
    let content = std::fs::read_to_string(&tmp).unwrap();
    assert!(content.contains("MikuMikuDance To After Effects (Camera)"));
    assert!(content.contains("VmdComp"));
    assert!(content.contains("VmdCam"));
    let _ = std::fs::remove_file(&tmp);
}

#[test]
fn test_output_jsx_from_vmd_coordinate_content() {
    // position=[1,2,3], distance=-45, 1920x1080, scale=20
    // AE X = 1*20 + 960 = 980, AE Y = -2*20 + 540 = 500, AE Z = 3*20 = 60
    // anchor_z = -(-45)*20 = 900
    let config = CameraJsxConfig {
        width: 1920, height: 1080, pixel_aspect: 1.0,
        fps: 30,
        comp_name: "Comp".to_string(),
        camera_name: "Cam".to_string(),
    };
    let tmp = std::env::temp_dir().join("mmd2jsx_test_vmd_coord.jsx");
    output_jsx_from_vmd(&[make_vmd_camera(0, [1.0, 2.0, 3.0], -45.0, 30)], &config, Some(&tmp)).unwrap();
    let content = std::fs::read_to_string(&tmp).unwrap();
    assert!(content.contains("980.000000"), "AE X not found in:\n{}", &content[..200.min(content.len())]);
    assert!(content.contains("500.000000"), "AE Y not found");
    assert!(content.contains("60.000000"),  "AE Z not found");
    assert!(content.contains("900.000000"), "anchor_z not found");
    let _ = std::fs::remove_file(&tmp);
}

// ────── output_jsx_from_pmm 公開 API ──────

fn make_linear_pmm_ci() -> PmmCameraInterp {
    PmmCameraInterp { ax: 20.0 / 127.0, ay: 20.0 / 127.0, bx: 107.0 / 127.0, by: 107.0 / 127.0 }
}

fn make_pmm_camera(frame: i32, position: [f32; 3], distance: f32, fov_deg: f32) -> PmmCameraFrame {
    let lin = make_linear_pmm_ci();
    PmmCameraFrame {
        frame,
        distance,
        position,
        rotation: [0.0, 0.0, 0.0],
        interp_x: lin, interp_y: lin, interp_z: lin,
        interp_rotation: lin, interp_distance: lin, interp_fov: lin,
        is_orth: false,
        fov_deg,
        follow_model: -1,
        follow_bone: -1,
    }
}

#[test]
fn test_output_jsx_from_pmm_single_frame() {
    let pmm_data = PmmData {
        output_width: 1920, output_height: 1080,
        models: vec![], cameras: vec![],
    };
    let config = CameraJsxConfig {
        width: 1920, height: 1080, pixel_aspect: 1.0,
        fps: 30,
        comp_name: "PmmComp".to_string(),
        camera_name: "PmmCam".to_string(),
    };
    let tmp = std::env::temp_dir().join("mmd2jsx_test_pmm_cam.jsx");
    output_jsx_from_pmm(
        &[make_pmm_camera(0, [0.0, 10.0, 0.0], -45.0, 30.0)],
        &pmm_data, &[], &config, Some(&tmp),
    ).unwrap();
    let content = std::fs::read_to_string(&tmp).unwrap();
    assert!(content.contains("MikuMikuDance To After Effects (Camera)"));
    assert!(content.contains("PmmComp"));
    assert!(content.contains("PmmCam"));
    let _ = std::fs::remove_file(&tmp);
}
