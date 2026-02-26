// PMMファイルパーサー
// MikuMikuDance プロジェクトファイル (.pmm) を読み込み、
// モデル・ボーンキーフレーム・カメラキーフレーム・外部親情報を提供する。

use super::reader::Reader;
use glam::{Quat, Vec3};
use std::io::{self, BufReader, Read};
use std::path::Path;
use std::fs::File;

// ─────────────────────────────────────────────
// データ構造
// ─────────────────────────────────────────────

/// ボーンキーフレームの補間曲線（4点のbyte値）
/// [x1, y1, x2, y2] で始点側(x1,y1)と終点側(x2,y2)の制御点を表す
#[derive(Debug, Clone)]
pub struct InterpolationCurve {
    pub x1: u8,
    pub y1: u8,
    pub x2: u8,
    pub y2: u8,
}

impl InterpolationCurve {
    /// 線形補間かどうか（制御点がデフォルト値 (20,20)-(107,107) に等しい場合）
    pub fn is_linear(&self) -> bool {
        self.x1 == self.y1 && self.x2 == self.y2
    }
}

#[derive(Debug, Clone)]
pub struct BoneFrame {
    /// フレーム番号
    pub frame: i32,
    /// X軸移動補間曲線
    pub interp_x: InterpolationCurve,
    /// Y軸移動補間曲線
    pub interp_y: InterpolationCurve,
    /// Z軸移動補間曲線
    pub interp_z: InterpolationCurve,
    /// 回転補間曲線
    pub interp_rot: InterpolationCurve,
    /// 移動量 (MMD座標系)
    pub movement: Vec3,
    /// 回転量 (クォータニオン)
    pub rotation: Quat,
    /// 物理有効フラグ
    pub physics_enabled: bool,
}

#[derive(Debug)]
pub struct PmmBone {
    pub name: String,
    pub frames: Vec<BoneFrame>,
}

/// 外部親エントリ（コンフィグフレームごと）
#[derive(Debug, Clone)]
pub struct ExternParentEntry {
    /// 参照先モデルのインデックス（PMMシーン内、-1=なし）
    pub model_index: i32,
    /// 参照先モデルのボーンインデックス（PMXボーンインデックスと同一、-1=なし）
    pub bone_index: i32,
}

/// フレームごとのIK有効/無効状態
#[derive(Debug, Clone)]
pub struct ConfigFrame {
    pub frame: i32,
    pub ik_enabled: Vec<bool>,          // ik_bone_indices に対応する有効/無効フラグ
    pub extern_parents: Vec<ExternParentEntry>, // parentable_bone_indices に対応する外部親情報
}

#[derive(Debug)]
pub struct PmmModel {
    pub name: String,
    /// PMMファイル内のモデルID（外部親参照で使われる可能性あり）
    pub model_id: u8,
    pub render_order: u8,
    /// PMXファイルへのパス（PMMファイルからの絶対パス）
    pub path: String,
    pub bones: Vec<PmmBone>,
    /// IKボーンのボーンインデックスリスト（PMXボーンインデックス）
    pub ik_bone_indices: Vec<i32>,
    /// 初期フレームのIK有効/無効
    pub initial_ik_state: Vec<bool>,
    /// フレームごとのIK状態（frame番号でソート済み）
    pub config_frames: Vec<ConfigFrame>,
    /// 外部親設定可能ボーンのPMXインデックスリスト
    pub parentable_bone_indices: Vec<i32>,
    /// 初期フレームの外部親状態（コンフィグフレームなし時のフォールバック）
    pub initial_extern_parents: Vec<ExternParentEntry>,
}

/// PMMカメラ補間パラメータ（1軸分、値域 0.0-1.0）
#[derive(Debug, Clone, Copy)]
pub struct PmmCameraInterp {
    pub ax: f32,
    pub ay: f32,
    pub bx: f32,
    pub by: f32,
}

/// PMMカメラキーフレーム（初期フレーム・キーフレーム共通）
#[derive(Debug, Clone)]
pub struct PmmCameraFrame {
    pub frame: i32,
    /// カメラ距離（負値）
    pub distance: f32,
    /// 注視点位置 [x, y, z]（追従ボーンがある場合はそこからの相対オフセット）
    pub position: [f32; 3],
    /// 角度 [x, y, z]（ラジアン）
    pub rotation: [f32; 3],
    pub interp_x:        PmmCameraInterp,
    pub interp_y:        PmmCameraInterp,
    pub interp_z:        PmmCameraInterp,
    pub interp_rotation: PmmCameraInterp,
    pub interp_distance: PmmCameraInterp,
    pub interp_fov:      PmmCameraInterp,
    /// 正射影フラグ
    pub is_orth: bool,
    /// 視野角（整数度）
    pub fov_deg: f32,
    /// ボーン追従モデルインデックス（0ベース配列インデックス、-1=追従なし）
    pub follow_model: i32,
    /// ボーン追従ボーンインデックス（PMXボーンインデックス、-1=追従なし）
    pub follow_bone: i32,
}

#[derive(Debug)]
pub struct PmmData {
    pub output_width: i32,
    pub output_height: i32,
    pub models: Vec<PmmModel>,
    pub cameras: Vec<PmmCameraFrame>,
}

// ─────────────────────────────────────────────
// PMMファイル読み込み
// ─────────────────────────────────────────────

pub fn read_pmm(path: &Path) -> io::Result<PmmData> {
    let file = File::open(path)?;
    let buf = BufReader::new(file);
    let mut r = Reader::new(buf);

    // ヘッダ
    let version = r.read_string_sjis_fixed(30)?;
    if version != "Polygon Movie maker 0002" {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("not a PMM file (version: {:?})", version),
        ));
    }

    let output_width = r.read_i32()?;
    let output_height = r.read_i32()?;
    let _editor_width = r.read_i32()?;
    let _view_angle = r.read_f32()?;
    let _is_camera_mode = r.read_bool()?;

    // パネル開閉状態 (6 bytes)
    let _ = r.read_bytes(6)?;

    // モデル読み込み
    let _selected_model_index = r.read_u8()?;
    let model_count = r.read_u8()?;

    let mut models = Vec::with_capacity(model_count as usize);
    for _ in 0..model_count {
        let model = read_model(&mut r)?;
        models.push(model);
    }

    // カメラセクション読み込み
    let cameras = read_cameras(&mut r)?;

    Ok(PmmData { output_width, output_height, models, cameras })
}

fn read_model<R: Read>(r: &mut Reader<R>) -> io::Result<PmmModel> {
    let model_id = r.read_u8()?;
    let name = r.read_dotnet_string()?;
    let _name_en = r.read_dotnet_string()?;
    let _path = r.read_string_sjis_fixed(256)?;  // Shift JIS固定長256バイト
    let _keyframe_editor_rows = r.read_u8()?;

    // ボーン名リスト
    let bone_count = r.read_i32()? as usize;
    let mut bone_names = Vec::with_capacity(bone_count);
    for _ in 0..bone_count {
        bone_names.push(r.read_dotnet_string()?);
    }

    // モーフ名リスト
    let morph_count = r.read_i32()? as usize;
    let _morph_names: Vec<String> = (0..morph_count)
        .map(|_| r.read_dotnet_string())
        .collect::<io::Result<_>>()?;

    // IKボーンインデックス
    let ik_count = r.read_i32()? as usize;
    let ik_bone_indices: Vec<i32> = (0..ik_count)
        .map(|_| r.read_i32())
        .collect::<io::Result<_>>()?;

    // 外部親設定可能ボーンインデックス（PMXボーンインデックス）
    let parentable_count = r.read_i32()? as usize;
    let parentable_bone_indices: Vec<i32> = (0..parentable_count)
        .map(|_| r.read_i32())
        .collect::<io::Result<_>>()?;

    let render_order = r.read_u8()?;
    let _visible = r.read_bool()?;

    // 選択ボーン/モーフインデックス (5 × i32)
    let _ = r.read_bytes(5 * 4)?;

    // ノード開閉状態
    let node_count = r.read_u8()? as usize;
    let _ = r.read_bytes(node_count)?;

    // エディタ状態
    let _vertical_scroll = r.read_i32()?;
    let _last_frame = r.read_i32()?;

    // 初期ボーンフレームの読み込み（各ボーン1つずつ）
    let mut init_next_ids: Vec<Option<i32>> = Vec::with_capacity(bone_count);
    let mut bones: Vec<PmmBone> = bone_names
        .iter()
        .map(|name| PmmBone {
            name: name.clone(),
            frames: Vec::new(),
        })
        .collect();

    for i in 0..bone_count {
        let (frame, _prev_id, next_id) = read_bone_frame_initial(r)?;
        bones[i].frames.push(frame);
        init_next_ids.push(if next_id == 0 { None } else { Some(next_id) });
    }

    // 非初期ボーンフレームの読み込み
    let bone_frame_count = r.read_i32()? as usize;
    let mut all_bone_frames: Vec<(i32, i32, BoneFrame)> = Vec::with_capacity(bone_frame_count);
    for _ in 0..bone_frame_count {
        let frame_index = r.read_i32()?;
        let (frame, _prev_id, next_id) = read_bone_frame_initial(r)?;
        all_bone_frames.push((frame_index, next_id, frame));
    }

    // フレームをボーンに割り当て（リンクリスト追跡）
    let frame_map: std::collections::HashMap<i32, usize> = all_bone_frames
        .iter()
        .enumerate()
        .filter_map(|(i, (idx, _, _))| if *idx != 0 { Some((*idx, i)) } else { None })
        .collect();

    for (bone_idx, next_id_opt) in init_next_ids.iter().enumerate() {
        let mut current_next = *next_id_opt;
        while let Some(next_id) = current_next {
            if let Some(&fi) = frame_map.get(&next_id) {
                let (_, next_next, frame) = &all_bone_frames[fi];
                bones[bone_idx].frames.push(frame.clone());
                current_next = if *next_next == 0 { None } else { Some(*next_next) };
            } else {
                break;
            }
        }
    }

    // フレームをframe番号でソート
    for bone in &mut bones {
        bone.frames.sort_by_key(|f| f.frame);
    }

    // モーフフレームの読み込み（スキップ）
    let morph_init_next_ids: Vec<Option<i32>> = (0..morph_count)
        .map(|_| {
            let (_frame, _prev, next) = read_morph_frame_initial(r)?;
            Ok(if next == 0 { None } else { Some(next) })
        })
        .collect::<io::Result<_>>()?;

    let morph_frame_count = r.read_i32()? as usize;
    // 各モーフフレーム: i32 (index) + i32 (frame) + i32 (prev) + i32 (next) + f32 (weight) + bool (selected)
    // = 4+4+4+4+4+1 = 21 bytes
    let _ = r.read_bytes(morph_frame_count * 21)?;
    let _ = morph_init_next_ids; // suppress warning

    // 初期コンフィグフレーム
    let initial_cf = read_config_frame(r, ik_count, parentable_count, true)?;
    let initial_ik_state = initial_cf.ik_enabled;
    let initial_extern_parents = initial_cf.extern_parents;

    // コンフィグフレーム
    let config_frame_count = r.read_i32()? as usize;
    let mut config_frames: Vec<ConfigFrame> = Vec::with_capacity(config_frame_count);
    for _ in 0..config_frame_count {
        let cf = read_config_frame(r, ik_count, parentable_count, false)?;
        config_frames.push(cf);
    }
    config_frames.sort_by_key(|cf| cf.frame);

    // 現在のボーン状態 (各ボーン: Vec3 + Quaternion + bool + bool + bool = 12+16+3 = 31 bytes)
    let _ = r.read_bytes(bone_count * 31)?;

    // 現在のモーフ状態 (各モーフ: f32 = 4 bytes)
    let _ = r.read_bytes(morph_count * 4)?;

    // 現在のIK状態 (各IKボーン: bool = 1 byte)
    let _ = r.read_bytes(ik_count)?;

    // 現在の外部親状態 (各親設定可能ボーン: i32+i32+i32+i32 = 16 bytes)
    let _ = r.read_bytes(parentable_count * 16)?;

    // 描画情報: bool + f32 + bool + byte = 7 bytes
    let _ = r.read_bytes(7)?;

    Ok(PmmModel {
        name,
        model_id,
        render_order,
        path: _path,
        bones,
        ik_bone_indices,
        initial_ik_state,
        config_frames,
        parentable_bone_indices,
        initial_extern_parents,
    })
}

// ─────────────────────────────────────────────
// カメラ読み込み
// ─────────────────────────────────────────────

fn read_cameras<R: Read>(r: &mut Reader<R>) -> io::Result<Vec<PmmCameraFrame>> {
    let mut cameras = Vec::new();

    // 初期フレーム（DataIndex なし）
    cameras.push(read_camera_frame(r, false)?);

    // キーフレーム数 + キーフレーム（DataIndex あり）
    let key_count = r.read_i32()? as usize;
    for _ in 0..key_count {
        cameras.push(read_camera_frame(r, true)?);
    }

    cameras.sort_by_key(|c| c.frame);
    Ok(cameras)
}

fn read_camera_frame<R: Read>(r: &mut Reader<R>, has_data_index: bool) -> io::Result<PmmCameraFrame> {
    if has_data_index { let _ = r.read_i32()?; }

    let frame          = r.read_i32()?;
    let _before_index  = r.read_i32()?;
    let _after_index   = r.read_i32()?;
    let distance       = r.read_f32()?;
    let position       = [r.read_f32()?, r.read_f32()?, r.read_f32()?];
    let rotation       = [r.read_f32()?, r.read_f32()?, r.read_f32()?];
    let follow_model   = r.read_i32()?;
    let follow_bone    = r.read_i32()?;

    let interp_x        = read_camera_interp(r)?;
    let interp_y        = read_camera_interp(r)?;
    let interp_z        = read_camera_interp(r)?;
    let interp_rotation = read_camera_interp(r)?;
    let interp_distance = read_camera_interp(r)?;
    let interp_fov      = read_camera_interp(r)?;

    let is_orth = r.read_bool()?;
    let fov_deg = r.read_i32()? as f32;
    let _selected = r.read_bool()?;

    Ok(PmmCameraFrame {
        frame, distance, position, rotation,
        interp_x, interp_y, interp_z, interp_rotation, interp_distance, interp_fov,
        is_orth, fov_deg,
        follow_model, follow_bone,
    })
}

/// カメラ補間データ（4バイト、各 0-127 → 0.0-1.0 正規化）
fn read_camera_interp<R: Read>(r: &mut Reader<R>) -> io::Result<PmmCameraInterp> {
    let bytes = r.read_bytes(4)?;
    const NORM: f32 = 127.0;
    Ok(PmmCameraInterp {
        ax: bytes[0] as f32 / NORM,
        ay: bytes[1] as f32 / NORM,
        bx: bytes[2] as f32 / NORM,
        by: bytes[3] as f32 / NORM,
    })
}

/// 初期ボーンフレームを読み込む（フレームインデックスなし）
/// 戻り値: (フレームデータ, 前フレームID, 次フレームID)
fn read_bone_frame_initial<R: Read>(r: &mut Reader<R>) -> io::Result<(BoneFrame, i32, i32)> {
    let frame_num = r.read_i32()?;
    let prev_id = r.read_i32()?;
    let next_id = r.read_i32()?;

    let interp_x   = read_interp_curve(r)?;
    let interp_y   = read_interp_curve(r)?;
    let interp_z   = read_interp_curve(r)?;
    let interp_rot = read_interp_curve(r)?;

    let movement = r.read_vec3()?;
    let rotation = r.read_quat()?;
    let _is_selected = r.read_bool()?;
    let physic_disabled = r.read_bool()?;

    let frame = BoneFrame {
        frame: frame_num,
        interp_x,
        interp_y,
        interp_z,
        interp_rot,
        movement,
        rotation,
        physics_enabled: !physic_disabled,
    };

    Ok((frame, prev_id, next_id))
}

/// 初期モーフフレームを読み込む（スキップ用）
fn read_morph_frame_initial<R: Read>(r: &mut Reader<R>) -> io::Result<(i32, i32, i32)> {
    let frame_num = r.read_i32()?;
    let prev_id = r.read_i32()?;
    let next_id = r.read_i32()?;
    let _weight = r.read_f32()?;
    let _is_selected = r.read_bool()?;
    Ok((frame_num, prev_id, next_id))
}

/// コンフィグフレームを読み込む
fn read_config_frame<R: Read>(
    r: &mut Reader<R>,
    ik_count: usize,
    parentable_count: usize,
    is_initial: bool,
) -> io::Result<ConfigFrame> {
    if !is_initial {
        let _ = r.read_i32()?; // frame index（リンクリスト用ID）
    }
    let frame_num = r.read_i32()?;
    let _prev_id = r.read_i32()?;
    let _next_id = r.read_i32()?;
    let _visible = r.read_bool()?;

    // IKの有効状態 (各IKボーン: bool)
    let ik_enabled: Vec<bool> = (0..ik_count)
        .map(|_| r.read_bool())
        .collect::<io::Result<_>>()?;

    // 外部親情報 (各ボーン: model_index i32 + bone_index i32 = 8 bytes)
    let extern_parents: Vec<ExternParentEntry> = (0..parentable_count)
        .map(|_| -> io::Result<ExternParentEntry> {
            Ok(ExternParentEntry {
                model_index: r.read_i32()?,
                bone_index:  r.read_i32()?,
            })
        })
        .collect::<io::Result<_>>()?;

    let _is_selected = r.read_bool()?;

    Ok(ConfigFrame { frame: frame_num, ik_enabled, extern_parents })
}

/// PMM固有: 補間曲線（4バイト）を読み込む
fn read_interp_curve<R: Read>(r: &mut Reader<R>) -> io::Result<InterpolationCurve> {
    let bytes = r.read_bytes(4)?;
    Ok(InterpolationCurve {
        x1: bytes[0],
        y1: bytes[1],
        x2: bytes[2],
        y2: bytes[3],
    })
}
