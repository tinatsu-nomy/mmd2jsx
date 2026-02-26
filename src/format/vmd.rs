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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::reader::Reader;
    use encoding_rs::SHIFT_JIS;
    use std::io::Cursor;

    /// 最小カメラVMDバイト列を構築するヘルパー
    fn make_camera_vmd(frames: &[(u32, f32, [f32; 3])]) -> Vec<u8> {
        let mut data = Vec::new();

        // ヘッダ: 30バイト
        let mut header = b"Vocaloid Motion Data 0002".to_vec();
        header.resize(30, 0);
        data.extend_from_slice(&header);

        // モデル名: 20バイト ("カメラ・照明" in SJIS)
        let (sjis, _, _) = SHIFT_JIS.encode("カメラ・照明");
        let mut model_name = sjis.to_vec();
        model_name.resize(20, 0);
        data.extend_from_slice(&model_name);

        // motion_count = 0, skin_count = 0
        data.extend_from_slice(&0u32.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes());

        // camera_count
        data.extend_from_slice(&(frames.len() as u32).to_le_bytes());

        // カメラフレーム: frame_no(4) + length(4) + location(12) + rotation(12) + interp(24) + viewing_angle(4) + perspective(1) = 61バイト
        for &(frame_no, length, loc) in frames {
            data.extend_from_slice(&frame_no.to_le_bytes());
            data.extend_from_slice(&length.to_le_bytes());
            for v in &loc { data.extend_from_slice(&v.to_le_bytes()); }
            // rotation (0, 0, 0)
            for _ in 0..3 { data.extend_from_slice(&0.0f32.to_le_bytes()); }
            // interpolation (24 bytes, linear MMD default)
            data.extend_from_slice(&[20, 107, 20, 107, 20, 107, 20, 107,
                                      20, 107, 20, 107, 20, 107, 20, 107,
                                      20, 107, 20, 107, 20, 107, 20, 107]);
            // viewing_angle = 30
            data.extend_from_slice(&30u32.to_le_bytes());
            // perspective = 0
            data.push(0);
        }
        data
    }

    fn make_reader(data: &[u8]) -> Reader<Cursor<Vec<u8>>> {
        Reader::new(Cursor::new(data.to_vec()))
    }

    #[test]
    fn test_read_vmd_invalid_header() {
        let mut data = vec![0u8; 50];
        data[..4].copy_from_slice(b"XVMD");
        let mut r = make_reader(&data);
        assert!(read_vmd_from(&mut r).is_err());
    }

    #[test]
    fn test_read_vmd_non_camera_model() {
        let mut data = Vec::new();
        let mut header = b"Vocaloid Motion Data 0002".to_vec();
        header.resize(30, 0);
        data.extend_from_slice(&header);
        // モデル名: "初音ミク" (not camera)
        let (sjis, _, _) = SHIFT_JIS.encode("初音ミク");
        let mut model_name = sjis.to_vec();
        model_name.resize(20, 0);
        data.extend_from_slice(&model_name);
        // motion_count, skin_count, camera_count
        for _ in 0..3 { data.extend_from_slice(&0u32.to_le_bytes()); }
        let mut r = make_reader(&data);
        assert!(read_vmd_from(&mut r).is_err());
    }

    #[test]
    fn test_read_vmd_single_frame() {
        let data = make_camera_vmd(&[(0, -45.0, [0.0, 10.0, 0.0])]);
        let mut r = make_reader(&data);
        let vmd = read_vmd_from(&mut r).unwrap();
        assert_eq!(vmd.cameras.len(), 1);
        assert_eq!(vmd.cameras[0].frame_no, 0);
        assert!((vmd.cameras[0].length - (-45.0)).abs() < 1e-5);
        assert!((vmd.cameras[0].location[1] - 10.0).abs() < 1e-5);
    }

    #[test]
    fn test_read_vmd_zero_cameras_error() {
        let data = make_camera_vmd(&[]);
        let mut r = make_reader(&data);
        assert!(read_vmd_from(&mut r).is_err());
    }

    #[test]
    fn test_read_vmd_frames_sorted() {
        // フレーム順: 10, 0, 5 → ソート後: 0, 5, 10
        let data = make_camera_vmd(&[
            (10, 0.0, [0.0; 3]),
            (0,  0.0, [0.0; 3]),
            (5,  0.0, [0.0; 3]),
        ]);
        let mut r = make_reader(&data);
        let vmd = read_vmd_from(&mut r).unwrap();
        assert_eq!(vmd.cameras.len(), 3);
        assert_eq!(vmd.cameras[0].frame_no, 0);
        assert_eq!(vmd.cameras[1].frame_no, 5);
        assert_eq!(vmd.cameras[2].frame_no, 10);
    }
}
