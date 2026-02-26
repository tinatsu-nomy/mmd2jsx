// カメラ変換・JSX出力モジュール
// VMD/PMMカメラデータをAfter Effects用JSXに変換する

use crate::format::vmd::VmdCamera;
use crate::format::pmm::{PmmCameraFrame, PmmCameraInterp, PmmData};
use crate::format::pmx::PmxBone;
use crate::jsx::bone::emit_jsx;
use crate::motion::interpolation;
use crate::motion::bezier::BezierCurve;
use std::io;
use std::path::Path;

// ─────────────────────────────────────────────
// ベジェ補間ヘルパー
// ─────────────────────────────────────────────

/// VMDカメラ補間配列から BezierCurve を生成（バイト順: [ax, bx, ay, by]、値域 0-127）
fn bezier_from_vmd(interp: &[u8; 24], offset: usize) -> BezierCurve {
    const N: f64 = 127.0;
    BezierCurve::new(
        interp[offset]     as f64 / N,   // ax
        interp[offset + 2] as f64 / N,   // ay
        interp[offset + 1] as f64 / N,   // bx
        interp[offset + 3] as f64 / N,   // by
    )
}

/// PMM補間パラメータから BezierCurve を生成
fn bezier_from_pmm_interp(c: &PmmCameraInterp) -> BezierCurve {
    BezierCurve::new(c.ax as f64, c.ay as f64, c.bx as f64, c.by as f64)
}

/// カメラ各パラメータのベジェ曲線セット
struct CameraInterpolation {
    x_move:   BezierCurve,
    y_move:   BezierCurve,
    z_move:   BezierCurve,
    rotation: BezierCurve,
    distance: BezierCurve,
    fov:      BezierCurve,
}

impl CameraInterpolation {
    fn from_vmd(interpolation: &[u8; 24]) -> Self {
        CameraInterpolation {
            x_move:   bezier_from_vmd(interpolation, 0),
            y_move:   bezier_from_vmd(interpolation, 4),
            z_move:   bezier_from_vmd(interpolation, 8),
            rotation: bezier_from_vmd(interpolation, 12),
            distance: bezier_from_vmd(interpolation, 16),
            fov:      bezier_from_vmd(interpolation, 20),
        }
    }

    fn from_pmm(cam: &PmmCameraFrame) -> Self {
        CameraInterpolation {
            x_move:   bezier_from_pmm_interp(&cam.interp_x),
            y_move:   bezier_from_pmm_interp(&cam.interp_y),
            z_move:   bezier_from_pmm_interp(&cam.interp_z),
            rotation: bezier_from_pmm_interp(&cam.interp_rotation),
            distance: bezier_from_pmm_interp(&cam.interp_distance),
            fov:      bezier_from_pmm_interp(&cam.interp_fov),
        }
    }
}

// ─────────────────────────────────────────────
// 内部共通表現
// ─────────────────────────────────────────────

/// VMD/PMM共通の内部カメラフレーム表現
struct CameraRaw {
    frame:    u32,
    position: [f64; 3],  // MMD座標系
    rotation: [f64; 3],  // ラジアン
    distance: f64,        // 負値
    fov_deg:  f64,        // 視野角（度）
    interp:   CameraInterpolation,
}

fn vmd_to_raw(cam: &VmdCamera) -> CameraRaw {
    CameraRaw {
        frame:    cam.frame_no,
        position: [cam.location[0] as f64, cam.location[1] as f64, cam.location[2] as f64],
        rotation: [cam.rotation[0] as f64, cam.rotation[1] as f64, cam.rotation[2] as f64],
        distance: cam.length as f64,
        fov_deg:  cam.viewing_angle as f64,
        interp:   CameraInterpolation::from_vmd(&cam.interpolation),
    }
}

fn pmm_to_raw(cam: &PmmCameraFrame) -> CameraRaw {
    CameraRaw {
        frame:    cam.frame as u32,
        position: [cam.position[0] as f64, cam.position[1] as f64, cam.position[2] as f64],
        rotation: [cam.rotation[0] as f64, cam.rotation[1] as f64, cam.rotation[2] as f64],
        distance: cam.distance as f64,
        fov_deg:  cam.fov_deg as f64,
        interp:   CameraInterpolation::from_pmm(cam),
    }
}

// ─────────────────────────────────────────────
// AEキーフレーム
// ─────────────────────────────────────────────

/// AEカメラキーフレーム（内部計算用）
struct AeCameraKeyframe {
    frame_no: u32,
    is_keyframe: bool,
    // 出力フラグ（前フレームから変化した場合のみtrue）
    out_position:   bool,
    out_y_rotation: bool,
    out_anchor_z:   bool,
    out_x_rotation: bool,
    out_z_rotation: bool,
    out_zoom:       bool,
    // 値
    position:   [f64; 3],
    y_rotation: f64,
    anchor_z:   f64,
    x_rotation: f64,
    z_rotation: f64,
    zoom:       f64,
}

// ─────────────────────────────────────────────
// 座標変換ヘルパー
// ─────────────────────────────────────────────

#[inline]
fn lerp(a: f64, b: f64, t: f64) -> f64 { a + (b - a) * t }

/// FOV（度）→ AE ズーム値
#[inline]
fn fov_to_zoom(fov_deg: f64, zoom_constant: f64) -> f64 {
    zoom_constant / (fov_deg / 2.0 * std::f64::consts::PI / 180.0).tan()
}

/// 6桁精度で等値かどうか（JSX出力精度での変化判定）
#[inline]
fn same_6dp(a: f64, b: f64) -> bool {
    (a * 1_000_000.0).round() == (b * 1_000_000.0).round()
}

// ─────────────────────────────────────────────
// 変換コアロジック
// ─────────────────────────────────────────────

/// 内部表現をAEキーフレームに変換（補間なし）
fn raw_to_ae(raw: &CameraRaw, width: u32, height: u32) -> AeCameraKeyframe {
    let zoom_constant = height as f64 / 2.0;
    let r2d = 180.0 / std::f64::consts::PI;
    AeCameraKeyframe {
        frame_no: raw.frame,
        is_keyframe: true,
        out_position: true, out_y_rotation: true, out_anchor_z: true,
        out_x_rotation: true, out_z_rotation: true, out_zoom: true,
        position: [
             raw.position[0] * 20.0 + width  as f64 / 2.0,
            -raw.position[1] * 20.0 + height as f64 / 2.0,
             raw.position[2] * 20.0,
        ],
        y_rotation: -raw.rotation[1] * r2d,
        anchor_z:   -raw.distance * 20.0,
        x_rotation:  raw.rotation[0] * r2d,
        z_rotation:  raw.rotation[2] * r2d,
        zoom: fov_to_zoom(raw.fov_deg, zoom_constant),
    }
}

/// 2フレーム間の補間セグメントをAEキーフレーム列に展開
fn interpolate_segment(
    start: &CameraRaw,
    end: &CameraRaw,
    width: u32,
    height: u32,
) -> Vec<AeCameraKeyframe> {
    let sf = start.frame;
    let ef = end.frame;
    let zoom_constant = height as f64 / 2.0;
    let r2d = 180.0 / std::f64::consts::PI;

    if ef <= sf + 1 {
        let mut v = vec![raw_to_ae(start, width, height)];
        if ef > sf { v.push(raw_to_ae(end, width, height)); }
        return v;
    }

    let interp = &end.interp;
    let out_pos  = !interp.x_move.is_linear() || !interp.y_move.is_linear() || !interp.z_move.is_linear();
    let out_rot  = !interp.rotation.is_linear();
    let out_dist = !interp.distance.is_linear();
    let out_zoom = !interp.fov.is_linear();

    if !out_pos && !out_rot && !out_dist && !out_zoom {
        return vec![raw_to_ae(start, width, height), raw_to_ae(end, width, height)];
    }

    let sp = start.position;
    let ep = end.position;
    let sr = start.rotation;
    let er = end.rotation;
    let sd = start.distance;
    let ed = end.distance;
    let sv = start.fov_deg;
    let ev = end.fov_deg;

    let mut result = Vec::new();
    let mut prev_pos:  Option<[f64; 3]> = None;
    let mut prev_yr:   Option<f64>      = None;
    let mut prev_xr:   Option<f64>      = None;
    let mut prev_zr:   Option<f64>      = None;
    let mut prev_az:   Option<f64>      = None;
    let mut prev_zoom: Option<f64>      = None;

    for frame in sf..=ef {
        let t = (frame - sf) as f64 / (ef - sf) as f64;
        let tx = interp.x_move.evaluate(t);
        let ty = interp.y_move.evaluate(t);
        let tz = interp.z_move.evaluate(t);
        let tr = interp.rotation.evaluate(t);
        let td = interp.distance.evaluate(t);
        let tv = interp.fov.evaluate(t);

        let pos = [
             lerp(sp[0], ep[0], tx) * 20.0 + width  as f64 / 2.0,
            -lerp(sp[1], ep[1], ty) * 20.0 + height as f64 / 2.0,
             lerp(sp[2], ep[2], tz) * 20.0,
        ];
        let yr = -lerp(sr[1], er[1], tr) * r2d;
        let xr =  lerp(sr[0], er[0], tr) * r2d;
        let zr =  lerp(sr[2], er[2], tr) * r2d;
        let az = -lerp(sd, ed, td) * 20.0;
        let zm =  fov_to_zoom(lerp(sv, ev, tv), zoom_constant);

        let is_kf = frame == sf || frame == ef;
        let pc  = out_pos  && prev_pos.map_or(true,  |p| !same_6dp(pos[0],p[0]) || !same_6dp(pos[1],p[1]) || !same_6dp(pos[2],p[2]));
        let yc  = out_rot  && prev_yr.map_or(true,   |p| !same_6dp(yr,  p));
        let xc  = out_rot  && prev_xr.map_or(true,   |p| !same_6dp(xr,  p));
        let zc  = out_rot  && prev_zr.map_or(true,   |p| !same_6dp(zr,  p));
        let ac  = out_dist && prev_az.map_or(true,   |p| !same_6dp(az,  p));
        let zmc = out_zoom && prev_zoom.map_or(true, |p| !same_6dp(zm,  p));

        prev_pos  = Some(pos);
        prev_yr   = Some(yr);
        prev_xr   = Some(xr);
        prev_zr   = Some(zr);
        prev_az   = Some(az);
        prev_zoom = Some(zm);

        if !is_kf && !pc && !yc && !xc && !zc && !ac && !zmc { continue; }

        result.push(AeCameraKeyframe {
            frame_no: frame, is_keyframe: is_kf,
            out_position: is_kf || pc, out_y_rotation: is_kf || yc,
            out_anchor_z: is_kf || ac, out_x_rotation: is_kf || xc,
            out_z_rotation: is_kf || zc, out_zoom: is_kf || zmc,
            position: pos, y_rotation: yr, anchor_z: az,
            x_rotation: xr, z_rotation: zr, zoom: zm,
        });
    }
    result
}

/// カメラフレーム列をAEキーフレームに変換
fn convert_cameras(raws: &[CameraRaw], width: u32, height: u32) -> Vec<AeCameraKeyframe> {
    if raws.is_empty() { return Vec::new(); }
    let mut result = vec![raw_to_ae(&raws[0], width, height)];
    for i in 0..raws.len() - 1 {
        let interpolated = interpolate_segment(&raws[i], &raws[i + 1], width, height);
        result.extend(interpolated.into_iter().skip(1));
    }
    result
}

// ─────────────────────────────────────────────
// 追従ボーン処理ヘルパー
// ─────────────────────────────────────────────

/// 特定フレームのカメラパラメータを補間して返す: (position, rotation, distance, fov_deg)
fn interp_camera_raw_at(raws: &[CameraRaw], frame: u32) -> ([f64; 3], [f64; 3], f64, f64) {
    if raws.is_empty() {
        return ([0.0; 3], [0.0; 3], 0.0, 30.0);
    }
    if frame <= raws[0].frame {
        let r = &raws[0];
        return (r.position, r.rotation, r.distance, r.fov_deg);
    }
    let last = raws.last().unwrap();
    if frame >= last.frame {
        return (last.position, last.rotation, last.distance, last.fov_deg);
    }
    let idx = raws.partition_point(|r| r.frame <= frame);
    let start = &raws[idx - 1];
    let end   = &raws[idx];

    let t  = (frame - start.frame) as f64 / (end.frame - start.frame) as f64;
    let ci = &end.interp;
    let tx = ci.x_move.evaluate(t);
    let ty = ci.y_move.evaluate(t);
    let tz = ci.z_move.evaluate(t);
    let tr = ci.rotation.evaluate(t);
    let td = ci.distance.evaluate(t);
    let tv = ci.fov.evaluate(t);

    (
        [
            lerp(start.position[0], end.position[0], tx),
            lerp(start.position[1], end.position[1], ty),
            lerp(start.position[2], end.position[2], tz),
        ],
        [
            lerp(start.rotation[0], end.rotation[0], tr),
            lerp(start.rotation[1], end.rotation[1], tr),
            lerp(start.rotation[2], end.rotation[2], tr),
        ],
        lerp(start.distance, end.distance, td),
        lerp(start.fov_deg,  end.fov_deg,  tv),
    )
}

/// 指定フレームでの追従設定を返す（直前のカメラキーフレームの値を使用）
fn get_follow_at(cameras: &[PmmCameraFrame], frame: i32) -> (i32, i32) {
    cameras.iter()
        .filter(|c| c.frame <= frame)
        .max_by_key(|c| c.frame)
        .map(|c| (c.follow_model, c.follow_bone))
        .unwrap_or((-1, -1))
}

/// 追従ボーンを考慮した全フレーム計算（追従あり時のみ呼ばれる）
fn convert_cameras_with_follow(
    cameras: &[PmmCameraFrame],
    pmm_data: &PmmData,
    all_pmx_bones: &[Option<Vec<PmxBone>>],
    width: u32,
    height: u32,
) -> Vec<AeCameraKeyframe> {
    if cameras.is_empty() { return Vec::new(); }

    let raws: Vec<CameraRaw> = cameras.iter().map(pmm_to_raw).collect();

    // フレーム範囲: カメラキーフレームと追従先モデルのボーンフレームの合算
    let first_frame = cameras[0].frame;
    let mut last_frame = cameras.last().unwrap().frame;

    for cam in cameras {
        if cam.follow_model < 0 { continue; }
        let idx = cam.follow_model as usize;
        if idx >= pmm_data.models.len() { continue; }
        let ref_model = &pmm_data.models[idx];
        if let Some(ref_last) = ref_model.bones.iter()
            .filter_map(|b| b.frames.last().map(|f| f.frame))
            .max()
        {
            last_frame = last_frame.max(ref_last);
        }
    }

    let zoom_constant = height as f64 / 2.0;
    let r2d = 180.0 / std::f64::consts::PI;

    // カメラキーフレームのフレーム番号セット
    let cam_kf_set: std::collections::HashSet<i32> = cameras.iter().map(|c| c.frame).collect();

    // 前フレームの値（変化判定用）
    struct Prev { pos: [f64; 3], yr: f64, az: f64, xr: f64, zr: f64, zm: f64 }
    let mut prev: Option<Prev> = None;
    let mut result: Vec<AeCameraKeyframe> = Vec::new();

    for frame_no in first_frame..=last_frame {
        let frame_u32 = frame_no as u32;

        // カメラパラメータ補間
        let (cam_pos, cam_rot, cam_dist, cam_fov) = interp_camera_raw_at(&raws, frame_u32);

        // 追従設定取得
        let (follow_model, follow_bone) = get_follow_at(cameras, frame_no);

        // 追従ボーンのワールド位置をオフセットとして加算
        let follow_offset: [f64; 3] = if follow_model >= 0 && follow_bone >= 0 {
            let model_arr_idx = follow_model as usize;
            if model_arr_idx < pmm_data.models.len() {
                if let Some(pos) = interpolation::get_bone_world_pos_at(
                    pmm_data, model_arr_idx, all_pmx_bones, follow_bone as usize, frame_no,
                ) {
                    [pos.x as f64, pos.y as f64, pos.z as f64]
                } else {
                    [0.0; 3]
                }
            } else {
                [0.0; 3]
            }
        } else {
            [0.0; 3]
        };

        // AE座標変換（追従オフセット加算後）
        let pos = [
             (cam_pos[0] + follow_offset[0]) * 20.0 + width  as f64 / 2.0,
            -(cam_pos[1] + follow_offset[1]) * 20.0 + height as f64 / 2.0,
             (cam_pos[2] + follow_offset[2]) * 20.0,
        ];
        let yr = -cam_rot[1] * r2d;
        let xr =  cam_rot[0] * r2d;
        let zr =  cam_rot[2] * r2d;
        let az = -cam_dist * 20.0;
        let zm =  fov_to_zoom(cam_fov, zoom_constant);

        let is_kf = cam_kf_set.contains(&frame_no);

        let (pc, yc, xc, zc, ac, zmc) = match &prev {
            None => (true, true, true, true, true, true),
            Some(p) => (
                !same_6dp(pos[0], p.pos[0]) || !same_6dp(pos[1], p.pos[1]) || !same_6dp(pos[2], p.pos[2]),
                !same_6dp(yr, p.yr),
                !same_6dp(xr, p.xr),
                !same_6dp(zr, p.zr),
                !same_6dp(az, p.az),
                !same_6dp(zm, p.zm),
            ),
        };

        prev = Some(Prev { pos, yr, az, xr, zr, zm });

        if !is_kf && !pc && !yc && !xc && !zc && !ac && !zmc { continue; }

        result.push(AeCameraKeyframe {
            frame_no:       frame_u32,
            is_keyframe:    is_kf,
            out_position:   is_kf || pc,
            out_y_rotation: is_kf || yc,
            out_anchor_z:   is_kf || ac,
            out_x_rotation: is_kf || xc,
            out_z_rotation: is_kf || zc,
            out_zoom:       is_kf || zmc,
            position:   pos,
            y_rotation: yr,
            anchor_z:   az,
            x_rotation: xr,
            z_rotation: zr,
            zoom:       zm,
        });
    }

    result
}

// ─────────────────────────────────────────────
// JSX出力
// ─────────────────────────────────────────────

/// カメラJSX出力の設定
pub struct CameraJsxConfig {
    pub width:        u32,
    pub height:       u32,
    pub pixel_aspect: f64,
    pub fps:          u32,
    pub comp_name:    String,
    pub camera_name:  String,
}

/// 初期ズーム値（45°垂直FOV基準、アスペクト比補正）
fn compute_init_zoom(width: u32, height: u32) -> f64 {
    let half_angle_rad = 22.5_f64.to_radians();
    let aspect = width as f64 / height as f64;
    let tan_w = half_angle_rad.tan() * aspect;
    let h_fov_deg = tan_w.atan() * 2.0 * (180.0 / std::f64::consts::PI);
    (width as f64 / 2.0) / (h_fov_deg / 2.0).to_radians().tan()
}

fn format_f(v: f64) -> String { format!("{:.6}", v) }

fn generate_jsx(keyframes: &[AeCameraKeyframe], config: &CameraJsxConfig) -> String {
    if keyframes.is_empty() { return String::new(); }

    let duration = keyframes.last().unwrap().frame_no as f64 / config.fps as f64
        + 1.0 / config.fps as f64;
    let init_zoom = compute_init_zoom(config.width, config.height);

    let mut s = String::new();
    s.push_str("//=================================================================\n");
    s.push_str("// MikuMikuDance To After Effects (Camera)\n");
    s.push_str("// Reference: (C) 2010 Programmed by 遊太郎\n");
    s.push_str("//=================================================================\n\n");
    s.push_str("//- Composition Settings ------------------------------------------\n");
    s.push_str(&format!("var Width       = {};\n", config.width));
    s.push_str(&format!("var Height      = {};\n", config.height));
    s.push_str(&format!("var AspectRatio = {};\n", format_f(config.pixel_aspect)));
    s.push_str(&format!("var Duration    = {};\n", format_f(duration)));
    s.push_str(&format!("var FPS         = {};\n", config.fps));
    s.push('\n');
    s.push_str("//- Add Layers ---------------------------------------------------\n");
    s.push_str(&format!(
        "var newComp  = app.project.items.addComp( \"{}\", Width, Height, AspectRatio, Duration, FPS );\n",
        config.comp_name
    ));
    s.push_str(&format!(
        "var layCam   = newComp.layers.addCamera( \"{}\", [ 0, 0 ] );\n",
        config.camera_name
    ));
    s.push_str("var layNullx = newComp.layers.addNull();\n");
    s.push_str("var layNully = newComp.layers.addNull();\n\n");

    s.push_str("//- Initialize Null Layer ----------------------------------------\n");
    s.push_str("layNully.name        = \"MMD CAMERA CONTROL Y\";\n");
    s.push_str("layNully.threeDLayer = true;\n");
    s.push_str("layNully.anchorPoint.setValue( [ 0.0, 0.0, 0.0 ] );\n");
    s.push_str("layNully.position.setValue( [ 0.0, 0.0, 0.0 ] );\n\n");

    s.push_str("//- Initialize Null Layer ----------------------------------------\n");
    s.push_str("layNullx.parent      = layNully;\n");
    s.push_str("layNullx.name        = \"MMD CAMERA CONTROL X\";\n");
    s.push_str("layNullx.threeDLayer = true;\n");
    s.push_str("layNullx.anchorPoint.setValue( [ 0.0, 0.0, 0.0 ] );\n");
    s.push_str("layNullx.position.setValue( [ 0.0, 0.0, 0.0 ] );\n\n");

    s.push_str("//- Initialize Camera Layer ---------------------------------------\n");
    s.push_str("layCam.parent = layNullx;\n");
    s.push_str("layCam.anchorPoint.setValue( [ 0.0, 0.0, 0.0 ] );\n");
    s.push_str("layCam.position.setValue( [ 0.0, 0.0, 0.0 ] );\n");
    s.push_str(&format!("layCam.property( \"zoom\" ).setValue( {} );\n", format_f(init_zoom)));
    s.push_str(&format!("layCam.property( \"focusDistance\" ).setValue( {} );\n", format_f(init_zoom)));
    s.push_str("layCam.property( \"aperture\" ).setValue( 25.309299 );\n\n");

    let mut pos_key:    u32 = 0;
    let mut anchor_key: u32 = 0;

    for kf in keyframes {
        if !kf.out_position && !kf.out_y_rotation && !kf.out_anchor_z
            && !kf.out_x_rotation && !kf.out_z_rotation && !kf.out_zoom { continue; }

        let t = kf.frame_no as f64 / config.fps as f64;
        if kf.is_keyframe {
            s.push_str("//- Keyframe ------------------------------------------------------\n");
        } else {
            s.push_str("//- Keyframe (Bezier) ----------------------------------------------\n");
        }

        if kf.out_position {
            pos_key += 1;
            s.push_str(&format!(
                "layNully.position.setValueAtTime( {:.8}, [ {}, {}, {} ] );\n",
                t, format_f(kf.position[0]), format_f(kf.position[1]), format_f(kf.position[2])
            ));
            s.push_str(&format!("layNully.position.setSpatialAutoBezierAtKey( {}, false );\n", pos_key));
        }
        if kf.out_y_rotation {
            s.push_str(&format!(
                "layNully.yRotation.setValueAtTime( {:.8}, {} );\n",
                t, format_f(kf.y_rotation)
            ));
        }
        if kf.out_anchor_z {
            anchor_key += 1;
            s.push_str(&format!(
                "layNullx.anchorPoint.setValueAtTime( {:.8},[ 0.0, 0.0, {} ] );\n",
                t, format_f(kf.anchor_z)
            ));
            s.push_str(&format!("layNullx.anchorPoint.setSpatialAutoBezierAtKey( {}, false );\n", anchor_key));
        }
        if kf.out_x_rotation {
            s.push_str(&format!(
                "layNullx.xRotation.setValueAtTime( {:.8}, {} );\n",
                t, format_f(kf.x_rotation)
            ));
        }
        if kf.out_z_rotation {
            s.push_str(&format!(
                "layNullx.zRotation.setValueAtTime( {:.8}, {} );\n",
                t, format_f(kf.z_rotation)
            ));
        }
        if kf.out_zoom {
            s.push_str(&format!(
                "layCam.property( \"zoom\" ).setValueAtTime( {:.8},{} );\n",
                t, format_f(kf.zoom)
            ));
        }
        s.push('\n');
    }

    s
}

// ─────────────────────────────────────────────
// 公開API
// ─────────────────────────────────────────────

/// VMDカメラを変換してJSXを出力（path=None で標準出力）
pub fn output_jsx_from_vmd(
    cameras: &[VmdCamera],
    config: &CameraJsxConfig,
    path: Option<&Path>,
) -> io::Result<()> {
    let raws: Vec<_> = cameras.iter().map(vmd_to_raw).collect();
    emit_jsx(generate_jsx(&convert_cameras(&raws, config.width, config.height), config), path)
}

/// PMMカメラを変換してJSXを出力（path=None で標準出力）
///
/// `all_pmx_bones`: 各モデルのPMXボーンリスト（ボーン追従計算に使用、None=PMX未ロード）
pub fn output_jsx_from_pmm(
    cameras: &[PmmCameraFrame],
    pmm_data: &PmmData,
    all_pmx_bones: &[Option<Vec<PmxBone>>],
    config: &CameraJsxConfig,
    path: Option<&Path>,
) -> io::Result<()> {
    let has_follow = cameras.iter().any(|c| c.follow_model >= 0);
    let keyframes = if has_follow {
        convert_cameras_with_follow(cameras, pmm_data, all_pmx_bones, config.width, config.height)
    } else {
        let raws: Vec<_> = cameras.iter().map(pmm_to_raw).collect();
        convert_cameras(&raws, config.width, config.height)
    };
    emit_jsx(generate_jsx(&keyframes, config), path)
}
