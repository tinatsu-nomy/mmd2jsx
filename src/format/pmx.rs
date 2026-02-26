/// PMXファイルローダー
/// 目的: ボーンの初期位置（レスト姿勢）と階層構造を取得する
use super::reader::Reader;
use encoding_rs::{UTF_16LE, UTF_8};
use glam::Vec3;
use std::io::{self, Read};
use std::path::Path;
use std::fs::File;
use std::io::BufReader;

// ─────────────────────────────────────────────
// データ構造
// ─────────────────────────────────────────────

/// IKリンクボーン（CCDアルゴリズムで回転させる対象）
#[derive(Debug, Clone)]
pub struct IkLink {
    pub bone_index: i32,
    pub angle_min: Option<[f32; 3]>,   // XYZ 最小角度制限（ラジアン）
    pub angle_max: Option<[f32; 3]>,   // XYZ 最大角度制限（ラジアン）
}

/// IK情報
#[derive(Debug, Clone)]
pub struct IkInfo {
    pub target_bone_index: i32,   // IKターゲットボーンインデックス（エフェクタ）
    pub loop_count: i32,
    pub limit_angle: f32,         // 1ループあたりの最大回転角度（ラジアン）
    pub links: Vec<IkLink>,       // links[0]=エフェクタ最近傍, links[last]=ルート近傍
}

/// PMXヘッダのグローバル設定
#[derive(Debug)]
struct PmxSettings {
    encoding: u8,          // 0=UTF-16LE, 1=UTF-8
    num_additional_uv: u8,
    size_vertex_index: u8,
    size_texture_index: u8,
    size_material_index: u8,
    size_bone_index: u8,
    size_morph_index: u8,
    size_body_index: u8,
}

/// PMXボーン（FK/IK計算に必要な情報）
#[derive(Debug, Clone)]
pub struct PmxBone {
    pub name: String,
    /// ボーンの初期位置（ワールド座標、MMD座標系）
    pub position: Vec3,
    /// 親ボーンインデックス（-1=親なし）
    pub parent_index: i32,
    /// 変形階層（小さいほど先に処理される）
    pub transform_order: i32,
    /// IK情報（bit5=1のボーンのみ）
    pub ik: Option<IkInfo>,
    /// 付与元ボーンインデックス（bit8/bit9が立っているとき）
    pub add_bone_index: Option<i32>,
    /// 付与率
    pub add_ratio: f32,
    /// ローカル付与フラグ（bit7）: true=親のローカル変形量を付与
    pub is_local_add: bool,
    /// 回転付与フラグ（bit8）
    pub is_add_rotation: bool,
    /// 移動付与フラグ（bit9）
    pub is_add_translation: bool,
}

/// PMXモデル（ボーン情報のみ）
#[derive(Debug)]
pub struct PmxModel {
    pub name: String,
    pub bones: Vec<PmxBone>,
}

// ─────────────────────────────────────────────
// PMXファイル読み込み
// ─────────────────────────────────────────────

pub fn read_pmx(path: &Path) -> io::Result<PmxModel> {
    let file = File::open(path)?;
    let buf = BufReader::new(file);
    let mut r = Reader::new(buf);

    // マジックナンバー確認
    let magic = r.read_bytes(4)?;
    if &magic != b"PMX " {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "not a PMX file"));
    }

    // バージョン
    let _version = r.read_f32()?;

    // グローバル設定
    let settings_len = r.read_u8()?;
    if settings_len != 8 {
        return Err(io::Error::new(io::ErrorKind::InvalidData,
            format!("unexpected settings size: {}", settings_len)));
    }
    let settings = PmxSettings {
        encoding:             r.read_u8()?,
        num_additional_uv:    r.read_u8()?,
        size_vertex_index:    r.read_u8()?,
        size_texture_index:   r.read_u8()?,
        size_material_index:  r.read_u8()?,
        size_bone_index:      r.read_u8()?,
        size_morph_index:     r.read_u8()?,
        size_body_index:      r.read_u8()?,
    };
    let enc = settings.encoding;

    // モデル情報 (name, nameEn, comment, commentEn) × 各1文字列
    let model_name = read_pmx_string(&mut r, enc)?;
    let _name_en   = read_pmx_string(&mut r, enc)?;
    let _comment   = read_pmx_string(&mut r, enc)?;
    let _comment_en = read_pmx_string(&mut r, enc)?;

    // 頂点セクション（スキップ）
    skip_vertices(&mut r, &settings)?;

    // 面セクション（スキップ）
    skip_faces(&mut r, &settings)?;

    // テクスチャセクション（スキップ）
    skip_textures(&mut r, enc)?;

    // 材質セクション（スキップ）
    skip_materials(&mut r, &settings, enc)?;

    // ボーンセクション（読み込み）
    let bones = read_bones(&mut r, &settings, enc)?;

    Ok(PmxModel {
        name: model_name,
        bones,
    })
}

// ─────────────────────────────────────────────
// PMX固有ヘルパー
// ─────────────────────────────────────────────

/// PMX形式の文字列: int32(バイト長) + バイト列
fn read_pmx_string<R: Read>(r: &mut Reader<R>, encoding: u8) -> io::Result<String> {
    let len = r.read_i32()? as usize;
    if len == 0 { return Ok(String::new()); }
    let bytes = r.read_bytes(len)?;
    let s = if encoding == 0 {
        let (decoded, _, _) = UTF_16LE.decode(&bytes);
        decoded.into_owned()
    } else {
        let (decoded, _, _) = UTF_8.decode(&bytes);
        decoded.into_owned()
    };
    Ok(s)
}

/// 可変長インデックスを読み込む（非頂点: signed）
fn read_index_signed<R: Read>(r: &mut Reader<R>, size: u8) -> io::Result<i32> {
    match size {
        1 => Ok(r.read_i8()? as i32),
        2 => Ok(r.read_i16()? as i32),
        4 => r.read_i32(),
        _ => Err(io::Error::new(io::ErrorKind::InvalidData, format!("invalid index size: {}", size))),
    }
}

// ─────────────────────────────────────────────
// 各セクションのスキップ処理
// ─────────────────────────────────────────────

fn skip_vertices<R: Read>(r: &mut Reader<R>, s: &PmxSettings) -> io::Result<()> {
    let count = r.read_i32()? as usize;
    for _ in 0..count {
        r.read_bytes(12)?;  // Position (Vector3)
        r.read_bytes(12)?;  // Normal (Vector3)
        r.read_bytes(8)?;   // UV (Vector2)
        r.read_bytes(16 * s.num_additional_uv as usize)?;  // AdditionalUVs
        let weight_type = r.read_u8()?;
        match weight_type {
            0 => { r.read_bytes(s.size_bone_index as usize)?; }
            1 => { r.read_bytes(s.size_bone_index as usize * 2)?; r.read_bytes(4)?; }
            2 => { r.read_bytes(s.size_bone_index as usize * 4)?; r.read_bytes(4 * 4)?; }
            3 => { r.read_bytes(s.size_bone_index as usize * 2)?; r.read_bytes(4)?; r.read_bytes(12 * 3)?; }
            4 => { r.read_bytes(s.size_bone_index as usize * 4)?; r.read_bytes(4 * 4)?; }
            _ => return Err(io::Error::new(io::ErrorKind::InvalidData,
                    format!("unknown weight type: {}", weight_type))),
        }
        r.read_bytes(4)?;  // EdgeScale
    }
    Ok(())
}

fn skip_faces<R: Read>(r: &mut Reader<R>, s: &PmxSettings) -> io::Result<()> {
    let vertex_count = r.read_i32()? as usize;
    r.read_bytes(vertex_count * s.size_vertex_index as usize)?;
    Ok(())
}

fn skip_textures<R: Read>(r: &mut Reader<R>, enc: u8) -> io::Result<()> {
    let count = r.read_i32()? as usize;
    for _ in 0..count { let _ = read_pmx_string(r, enc)?; }
    Ok(())
}

fn skip_materials<R: Read>(r: &mut Reader<R>, s: &PmxSettings, enc: u8) -> io::Result<()> {
    let count = r.read_i32()? as usize;
    for _ in 0..count {
        let _ = read_pmx_string(r, enc)?;
        let _ = read_pmx_string(r, enc)?;
        r.read_bytes(16 + 12 + 4 + 12)?;  // Diffuse, Specular, Ambient
        r.read_bytes(1)?;                  // DrawFlag
        r.read_bytes(16 + 4)?;             // EdgeColor, EdgeWidth
        r.read_bytes(s.size_texture_index as usize)?;  // TextureIndex
        r.read_bytes(s.size_texture_index as usize)?;  // SphereMapIndex
        let _sphere_mode = r.read_u8()?;
        let is_toon_shared = r.read_u8()?;
        if is_toon_shared != 0 {
            r.read_bytes(1)?;
        } else {
            r.read_bytes(s.size_texture_index as usize)?;
        }
        let _ = read_pmx_string(r, enc)?;  // Memo
        r.read_bytes(4)?;                  // FaceCount
    }
    Ok(())
}

// ─────────────────────────────────────────────
// ボーン読み込み
// ─────────────────────────────────────────────

fn read_bones<R: Read>(r: &mut Reader<R>, s: &PmxSettings, enc: u8) -> io::Result<Vec<PmxBone>> {
    let count = r.read_i32()? as usize;
    let mut bones = Vec::with_capacity(count);

    for _ in 0..count {
        let name     = read_pmx_string(r, enc)?;
        let _name_en = read_pmx_string(r, enc)?;
        let position = r.read_vec3()?;
        let parent_index = read_index_signed(r, s.size_bone_index)?;
        let transform_order = r.read_i32()?;
        let flags = r.read_u16()?;

        // 接続先タイプ (bit0: 0=オフセット, 1=ボーンインデックス)
        if flags & 0x0001 != 0 {
            r.read_bytes(s.size_bone_index as usize)?;  // TargetBoneIndex
        } else {
            r.read_bytes(12)?;  // Offset Vector3
        }

        // ローカル付与(bit7) / 回転付与(bit8) / 移動付与(bit9)
        let is_local_add = flags & 0x0080 != 0;
        let (add_bone_index, add_ratio, is_add_rotation, is_add_translation) =
            if flags & 0x0100 != 0 || flags & 0x0200 != 0 {
                let add_idx = read_index_signed(r, s.size_bone_index)?;
                let ratio = r.read_f32()?;
                (Some(add_idx), ratio, flags & 0x0100 != 0, flags & 0x0200 != 0)
            } else {
                (None, 0.0f32, false, false)
            };

        // 軸固定 (bit10)
        if flags & 0x0400 != 0 { r.read_bytes(12)?; }

        // ローカル軸 (bit11)
        if flags & 0x0800 != 0 { r.read_bytes(24)?; }

        // 外部親 (bit13)
        if flags & 0x2000 != 0 { r.read_bytes(4)?; }

        // IK (bit5)
        let ik = if flags & 0x0020 != 0 {
            let target_bone_index = read_index_signed(r, s.size_bone_index)?;
            let loop_count = r.read_i32()?;
            let limit_angle = r.read_f32()?;
            let link_num = r.read_i32()? as usize;
            let mut links = Vec::with_capacity(link_num);
            for _ in 0..link_num {
                let link_bone = read_index_signed(r, s.size_bone_index)?;
                let enable_limit = r.read_u8()?;
                let (angle_min, angle_max) = if enable_limit != 0 {
                    let min = [r.read_f32()?, r.read_f32()?, r.read_f32()?];
                    let max = [r.read_f32()?, r.read_f32()?, r.read_f32()?];
                    (Some(min), Some(max))
                } else {
                    (None, None)
                };
                links.push(IkLink { bone_index: link_bone, angle_min, angle_max });
            }
            Some(IkInfo { target_bone_index, loop_count, limit_angle, links })
        } else {
            None
        };

        bones.push(PmxBone {
            name,
            position,
            parent_index,
            transform_order,
            ik,
            add_bone_index,
            add_ratio,
            is_local_add,
            is_add_rotation,
            is_add_translation,
        });
    }

    Ok(bones)
}
