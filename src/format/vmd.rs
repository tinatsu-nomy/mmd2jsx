// VMDファイル読み込みモジュール（カメラデータのみ）

use super::reader::Reader;
use std::fs::File;
use std::io::{self, BufReader, Read, Seek};
use std::path::Path;

// ─────────────────────────────────────────────
// データ構造
// ─────────────────────────────────────────────

pub struct VmdData {
    pub model_name: String,
    pub cameras: Vec<VmdCamera>,
}

pub struct VmdCamera {
    pub frame_no: u32,
    /// 距離（負値）
    pub length: f32,
    /// 位置 [x, y, z]
    pub location: [f32; 3],
    /// 回転（オイラー角）[x, y, z]（ラジアン）
    pub rotation: [f32; 3],
    /// 補間パラメータ（24バイト）
    pub interpolation: [u8; 24],
    /// 視野角（整数度）
    pub viewing_angle: u32,
    /// 透視投影フラグ（0=on, 1=off）
    pub perspective: u8,
}

// ─────────────────────────────────────────────
// 読み込み
// ─────────────────────────────────────────────

pub fn read_vmd(path: &Path) -> io::Result<VmdData> {
    let file = File::open(path)?;
    let mut reader = Reader::new(BufReader::new(file));
    read_vmd_from(&mut reader)
}

fn read_vmd_from<R: Read + Seek>(r: &mut Reader<R>) -> io::Result<VmdData> {
    // ヘッダ（30バイト）
    let header = r.read_string_sjis_fixed(30)?;
    if !header.starts_with("Vocaloid Motion Data 0002") {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("not a VMD file (header: {:?})", header),
        ));
    }

    // モデル名（20バイト、Shift-JIS）
    let model_name = r.read_string_sjis_fixed(20)?;

    // カメラVMDかどうか確認
    if model_name != "カメラ・照明" {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("not a camera VMD file (model name: '{}')", model_name),
        ));
    }

    // モーションデータ数（u32）→ スキップ（111バイト×個数）
    let motion_count = r.read_u32()?;
    r.seek_by(111 * motion_count as i64)?;

    // 表情データ数（u32）→ スキップ（23バイト×個数）
    let skin_count = r.read_u32()?;
    r.seek_by(23 * skin_count as i64)?;

    // カメラデータ数（u32）
    let camera_count = r.read_u32()?;
    if camera_count == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "no camera data (count = 0)",
        ));
    }

    // カメラフレーム読み込み（61バイト×個数）
    let mut cameras = Vec::with_capacity(camera_count as usize);
    for _ in 0..camera_count {
        let frame_no      = r.read_u32()?;
        let length        = r.read_f32()?;
        let location      = [r.read_f32()?, r.read_f32()?, r.read_f32()?];
        let rotation      = [r.read_f32()?, r.read_f32()?, r.read_f32()?];
        let interpolation = read_bytes24(r)?;
        let viewing_angle = r.read_u32()?;
        let perspective   = r.read_u8()?;
        cameras.push(VmdCamera { frame_no, length, location, rotation, interpolation, viewing_angle, perspective });
    }

    cameras.sort_by_key(|c| c.frame_no);

    Ok(VmdData { model_name, cameras })
}

fn read_bytes24<R: Read>(r: &mut Reader<R>) -> io::Result<[u8; 24]> {
    let bytes = r.read_bytes(24)?;
    let mut arr = [0u8; 24];
    arr.copy_from_slice(&bytes);
    Ok(arr)
}
