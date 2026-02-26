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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::reader::Reader;
    use std::io::Cursor;

    fn reader_from(data: &[u8]) -> Reader<Cursor<Vec<u8>>> {
        Reader::new(Cursor::new(data.to_vec()))
    }

    // ────── read_index_signed ──────

    #[test]
    fn test_read_index_signed_i8() {
        let mut r = reader_from(&[0xFE]);
        assert_eq!(read_index_signed(&mut r, 1).unwrap(), -2i32);
    }

    #[test]
    fn test_read_index_signed_i16() {
        let mut r = reader_from(&[5, 0]);
        assert_eq!(read_index_signed(&mut r, 2).unwrap(), 5i32);
    }

    #[test]
    fn test_read_index_signed_i32() {
        let bytes = (-1i32).to_le_bytes();
        let mut r = reader_from(&bytes);
        assert_eq!(read_index_signed(&mut r, 4).unwrap(), -1i32);
    }

    #[test]
    fn test_read_index_signed_invalid_size() {
        let mut r = reader_from(&[1, 2, 3]);
        assert!(read_index_signed(&mut r, 3).is_err());
    }

    // ────── read_pmx_string ──────

    #[test]
    fn test_read_pmx_string_utf8() {
        let mut data = Vec::new();
        data.extend_from_slice(&4i32.to_le_bytes()); // byte length = 4
        data.extend_from_slice(b"bone");
        let mut r = reader_from(&data);
        assert_eq!(read_pmx_string(&mut r, 1).unwrap(), "bone");
    }

    #[test]
    fn test_read_pmx_string_utf16le() {
        let mut data = Vec::new();
        data.extend_from_slice(&4i32.to_le_bytes()); // byte length = 4
        // "AB" in UTF-16LE
        data.extend_from_slice(&[0x41, 0x00, 0x42, 0x00]);
        let mut r = reader_from(&data);
        assert_eq!(read_pmx_string(&mut r, 0).unwrap(), "AB");
    }

    #[test]
    fn test_read_pmx_string_empty() {
        let mut data = Vec::new();
        data.extend_from_slice(&0i32.to_le_bytes()); // byte length = 0
        let mut r = reader_from(&data);
        assert_eq!(read_pmx_string(&mut r, 1).unwrap(), "");
    }

    // ────── skip_vertices ──────

    fn default_settings() -> PmxSettings {
        PmxSettings {
            encoding: 1, num_additional_uv: 0,
            size_vertex_index: 1, size_texture_index: 1,
            size_material_index: 1, size_bone_index: 1,
            size_morph_index: 1, size_body_index: 1,
        }
    }

    #[test]
    fn test_skip_vertices_zero() {
        let data = 0i32.to_le_bytes();
        let mut r = reader_from(&data);
        assert!(skip_vertices(&mut r, &default_settings()).is_ok());
    }

    #[test]
    fn test_skip_vertices_bdef1() {
        // count=1, weight_type=0 (BDEF1)
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes()); // count=1
        data.extend_from_slice(&[0u8; 12]); // position
        data.extend_from_slice(&[0u8; 12]); // normal
        data.extend_from_slice(&[0u8; 8]);  // UV
        // no additional UV
        data.push(0); // weight_type = BDEF1(0)
        data.push(0); // bone_index (size_bone_index=1)
        data.extend_from_slice(&[0u8; 4]); // EdgeScale
        let mut r = reader_from(&data);
        assert!(skip_vertices(&mut r, &default_settings()).is_ok());
    }

    #[test]
    fn test_skip_vertices_bdef2() {
        // count=1, weight_type=1 (BDEF2)
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes()); // count=1
        data.extend_from_slice(&[0u8; 32]); // pos(12)+normal(12)+UV(8)
        data.push(1); // weight_type = BDEF2(1)
        data.extend_from_slice(&[0u8; 2]); // bone_index × 2 (size=1)
        data.extend_from_slice(&[0u8; 4]); // weight f32
        data.extend_from_slice(&[0u8; 4]); // EdgeScale
        let mut r = reader_from(&data);
        assert!(skip_vertices(&mut r, &default_settings()).is_ok());
    }

    #[test]
    fn test_skip_vertices_bdef4() {
        // count=1, weight_type=2 (BDEF4)
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes()); // count=1
        data.extend_from_slice(&[0u8; 32]); // pos+normal+UV
        data.push(2); // weight_type = BDEF4(2)
        data.extend_from_slice(&[0u8; 4]); // bone_index × 4 (size=1)
        data.extend_from_slice(&[0u8; 16]); // weights 4×f32
        data.extend_from_slice(&[0u8; 4]); // EdgeScale
        let mut r = reader_from(&data);
        assert!(skip_vertices(&mut r, &default_settings()).is_ok());
    }

    #[test]
    fn test_skip_vertices_sdef() {
        // count=1, weight_type=3 (SDEF)
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes());
        data.extend_from_slice(&[0u8; 32]); // pos+normal+UV
        data.push(3); // weight_type = SDEF(3)
        data.extend_from_slice(&[0u8; 2]); // bone_index × 2
        data.extend_from_slice(&[0u8; 4]); // weight f32
        data.extend_from_slice(&[0u8; 36]); // C,R0,R1 (3×Vec3 = 36 bytes)
        data.extend_from_slice(&[0u8; 4]); // EdgeScale
        let mut r = reader_from(&data);
        assert!(skip_vertices(&mut r, &default_settings()).is_ok());
    }

    #[test]
    fn test_skip_vertices_qdef() {
        // count=1, weight_type=4 (QDEF)
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes());
        data.extend_from_slice(&[0u8; 32]); // pos+normal+UV
        data.push(4); // weight_type = QDEF(4)
        data.extend_from_slice(&[0u8; 4]); // bone_index × 4 (size=1)
        data.extend_from_slice(&[0u8; 16]); // weights 4×f32
        data.extend_from_slice(&[0u8; 4]); // EdgeScale
        let mut r = reader_from(&data);
        assert!(skip_vertices(&mut r, &default_settings()).is_ok());
    }

    // ────── skip_faces ──────

    #[test]
    fn test_skip_faces_zero() {
        let data = 0i32.to_le_bytes();
        let mut r = reader_from(&data);
        assert!(skip_faces(&mut r, &default_settings()).is_ok());
    }

    #[test]
    fn test_skip_faces_three_u16_indices() {
        // vertex_count=3, size_vertex_index=2 → 3×2=6 bytes
        let mut data = Vec::new();
        data.extend_from_slice(&3i32.to_le_bytes());
        data.extend_from_slice(&[0u8; 6]);
        let s = PmxSettings { size_vertex_index: 2, ..default_settings() };
        let mut r = reader_from(&data);
        assert!(skip_faces(&mut r, &s).is_ok());
    }

    // ────── skip_textures ──────

    #[test]
    fn test_skip_textures_zero() {
        let data = 0i32.to_le_bytes();
        let mut r = reader_from(&data);
        assert!(skip_textures(&mut r, 1).is_ok());
    }

    #[test]
    fn test_skip_textures_one_utf8() {
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes()); // count=1
        data.extend_from_slice(&3i32.to_le_bytes()); // length=3
        data.extend_from_slice(b"tex");
        let mut r = reader_from(&data);
        assert!(skip_textures(&mut r, 1).is_ok());
    }

    // ────── read_bones ──────

    #[test]
    fn test_read_bones_zero() {
        let data = 0i32.to_le_bytes();
        let mut r = reader_from(&data);
        let bones = read_bones(&mut r, &default_settings(), 1).unwrap();
        assert!(bones.is_empty());
    }

    #[test]
    fn test_read_bones_one_simple() {
        // 最小ボーン: name="B", name_en="", pos=(1,2,3),
        // parent=-1, transform_order=0, flags=0x0000 (offset connection)
        let s = default_settings(); // size_bone_index=1
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes()); // bone count=1
        // name "B" (UTF-8)
        data.extend_from_slice(&1i32.to_le_bytes());
        data.push(b'B');
        // name_en ""
        data.extend_from_slice(&0i32.to_le_bytes());
        // position (1.0, 2.0, 3.0)
        data.extend_from_slice(&1.0f32.to_le_bytes());
        data.extend_from_slice(&2.0f32.to_le_bytes());
        data.extend_from_slice(&3.0f32.to_le_bytes());
        // parent_index = -1 (i8)
        data.push(0xFF_u8);
        // transform_order = 0
        data.extend_from_slice(&0i32.to_le_bytes());
        // flags = 0x0000 (bit0=0: offset connection, no optional features)
        data.extend_from_slice(&0u16.to_le_bytes());
        // offset Vec3 (bit0=0)
        data.extend_from_slice(&[0u8; 12]);
        let mut r = reader_from(&data);
        let bones = read_bones(&mut r, &s, 1).unwrap();
        assert_eq!(bones.len(), 1);
        assert_eq!(bones[0].name, "B");
        assert!((bones[0].position.x - 1.0).abs() < 1e-6);
        assert!((bones[0].position.y - 2.0).abs() < 1e-6);
        assert!((bones[0].position.z - 3.0).abs() < 1e-6);
        assert_eq!(bones[0].parent_index, -1);
        assert!(bones[0].ik.is_none());
        assert!(!bones[0].is_add_rotation);
        assert!(!bones[0].is_add_translation);
    }

    #[test]
    fn test_read_bones_with_ik() {
        // flags: bit0=1 (bone connection), bit5=1 (IK)
        // flags = 0x0021
        let s = default_settings();
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes()); // count=1
        // name "IK" (UTF-8)
        data.extend_from_slice(&2i32.to_le_bytes());
        data.extend_from_slice(b"IK");
        // name_en ""
        data.extend_from_slice(&0i32.to_le_bytes());
        // position (0,0,0)
        data.extend_from_slice(&[0u8; 12]);
        // parent=-1
        data.push(0xFF_u8);
        // transform_order=0
        data.extend_from_slice(&0i32.to_le_bytes());
        // flags = 0x0021 (bit0=1: bone connection, bit5=1: IK)
        data.extend_from_slice(&0x0021u16.to_le_bytes());
        // target bone index (bit0=1 → bone index, size=1)
        data.push(1u8); // target = bone 1
        // IK: target_bone_index(1), loop_count(4), limit_angle(4), link_num(4), links
        data.push(0u8); // target_bone_index as i8
        data.extend_from_slice(&10i32.to_le_bytes()); // loop_count=10
        data.extend_from_slice(&1.0f32.to_le_bytes()); // limit_angle
        data.extend_from_slice(&1i32.to_le_bytes()); // link_num=1
        // link: bone_index(1), enable_limit(1)
        data.push(0u8); // link bone=0
        data.push(0u8); // enable_limit=false
        let mut r = reader_from(&data);
        let bones = read_bones(&mut r, &s, 1).unwrap();
        assert_eq!(bones.len(), 1);
        assert!(bones[0].ik.is_some());
        let ik = bones[0].ik.as_ref().unwrap();
        assert_eq!(ik.loop_count, 10);
        assert_eq!(ik.links.len(), 1);
        assert!(ik.links[0].angle_min.is_none());
    }

    #[test]
    fn test_read_bones_with_grant_rotation() {
        // flags: bit0=0 (offset), bit8=1 (回転付与) = 0x0100
        let s = default_settings();
        let mut data = Vec::new();
        data.extend_from_slice(&1i32.to_le_bytes()); // count=1
        // name "G"
        data.extend_from_slice(&1i32.to_le_bytes());
        data.push(b'G');
        // name_en ""
        data.extend_from_slice(&0i32.to_le_bytes());
        // position
        data.extend_from_slice(&[0u8; 12]);
        // parent=-1
        data.push(0xFF_u8);
        // transform_order
        data.extend_from_slice(&0i32.to_le_bytes());
        // flags = 0x0100 (bit8: 回転付与)
        data.extend_from_slice(&0x0100u16.to_le_bytes());
        // offset Vec3 (bit0=0)
        data.extend_from_slice(&[0u8; 12]);
        // add_bone_index (i8=0) + add_ratio (f32=0.5)
        data.push(0u8);
        data.extend_from_slice(&0.5f32.to_le_bytes());
        let mut r = reader_from(&data);
        let bones = read_bones(&mut r, &s, 1).unwrap();
        assert!(bones[0].is_add_rotation);
        assert!(!bones[0].is_add_translation);
        assert_eq!(bones[0].add_bone_index, Some(0));
        assert!((bones[0].add_ratio - 0.5).abs() < 1e-6);
    }
}

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
