/// 共通バイナリ読み込みユーティリティ
/// pmm.rs / pmx.rs / vmd.rs で共有する Reader<R> 実装

use encoding_rs::SHIFT_JIS;
use glam::{Quat, Vec3};
use std::io::{self, Read, Seek, SeekFrom};

pub(crate) struct Reader<R: Read> {
    pub(crate) inner: R,
}

impl<R: Read> Reader<R> {
    pub(crate) fn new(inner: R) -> Self { Self { inner } }

    pub(crate) fn read_u8(&mut self) -> io::Result<u8> {
        let mut buf = [0u8; 1];
        self.inner.read_exact(&mut buf)?;
        Ok(buf[0])
    }

    pub(crate) fn read_i8(&mut self) -> io::Result<i8> {
        Ok(self.read_u8()? as i8)
    }

    pub(crate) fn read_i16(&mut self) -> io::Result<i16> {
        let mut buf = [0u8; 2];
        self.inner.read_exact(&mut buf)?;
        Ok(i16::from_le_bytes(buf))
    }

    pub(crate) fn read_u16(&mut self) -> io::Result<u16> {
        let mut buf = [0u8; 2];
        self.inner.read_exact(&mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }

    pub(crate) fn read_i32(&mut self) -> io::Result<i32> {
        let mut buf = [0u8; 4];
        self.inner.read_exact(&mut buf)?;
        Ok(i32::from_le_bytes(buf))
    }

    pub(crate) fn read_u32(&mut self) -> io::Result<u32> {
        let mut buf = [0u8; 4];
        self.inner.read_exact(&mut buf)?;
        Ok(u32::from_le_bytes(buf))
    }

    pub(crate) fn read_f32(&mut self) -> io::Result<f32> {
        let mut buf = [0u8; 4];
        self.inner.read_exact(&mut buf)?;
        Ok(f32::from_le_bytes(buf))
    }

    pub(crate) fn read_bool(&mut self) -> io::Result<bool> {
        Ok(self.read_u8()? != 0)
    }

    pub(crate) fn read_bytes(&mut self, n: usize) -> io::Result<Vec<u8>> {
        let mut buf = vec![0u8; n];
        self.inner.read_exact(&mut buf)?;
        Ok(buf)
    }

    /// 固定長バイト列を Shift JIS として読み込む（\0 以降を切り捨て）
    pub(crate) fn read_string_sjis_fixed(&mut self, len: usize) -> io::Result<String> {
        let bytes = self.read_bytes(len)?;
        let null_pos = bytes.iter().position(|&b| b == 0).unwrap_or(len);
        let (decoded, _, _) = SHIFT_JIS.decode(&bytes[..null_pos]);
        Ok(decoded.into_owned())
    }

    /// .NET BinaryReader.ReadString() 互換: 7ビット符号化長プレフィクス + Shift JIS
    pub(crate) fn read_dotnet_string(&mut self) -> io::Result<String> {
        let byte_count = self.read_7bit_encoded_int()? as usize;
        if byte_count == 0 { return Ok(String::new()); }
        let bytes = self.read_bytes(byte_count)?;
        let (decoded, _, _) = SHIFT_JIS.decode(&bytes);
        Ok(decoded.into_owned())
    }

    /// C# の 7ビット符号化整数（BinaryReader.Read7BitEncodedInt）
    pub(crate) fn read_7bit_encoded_int(&mut self) -> io::Result<u32> {
        let mut result: u32 = 0;
        let mut shift = 0;
        loop {
            let byte = self.read_u8()?;
            result |= ((byte & 0x7F) as u32) << shift;
            shift += 7;
            if byte & 0x80 == 0 { break; }
            if shift >= 35 {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "7bit encoded int too long"));
            }
        }
        Ok(result)
    }

    /// Vec3（f32×3: x, y, z）を glam::Vec3 として読み込む
    pub(crate) fn read_vec3(&mut self) -> io::Result<Vec3> {
        Ok(Vec3::new(self.read_f32()?, self.read_f32()?, self.read_f32()?))
    }

    /// クォータニオン（f32×4: x, y, z, w）を glam::Quat として読み込む
    pub(crate) fn read_quat(&mut self) -> io::Result<Quat> {
        let x = self.read_f32()?;
        let y = self.read_f32()?;
        let z = self.read_f32()?;
        let w = self.read_f32()?;
        Ok(Quat::from_xyzw(x, y, z, w).normalize())
    }
}

impl<R: Read + Seek> Reader<R> {
    /// 現在位置から n バイト先にシークする
    pub(crate) fn seek_by(&mut self, n: i64) -> io::Result<()> {
        self.inner.seek(SeekFrom::Current(n))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn reader_from(data: &[u8]) -> Reader<Cursor<Vec<u8>>> {
        Reader::new(Cursor::new(data.to_vec()))
    }

    #[test]
    fn test_read_u8() {
        let mut r = reader_from(&[0xAB]);
        assert_eq!(r.read_u8().unwrap(), 0xAB);
    }

    #[test]
    fn test_read_i8_negative() {
        let mut r = reader_from(&[0xFF]);
        assert_eq!(r.read_i8().unwrap(), -1i8);
    }

    #[test]
    fn test_read_i16_le() {
        let mut r = reader_from(&[0x34, 0x12]);
        assert_eq!(r.read_i16().unwrap(), 0x1234i16);
    }

    #[test]
    fn test_read_u16_le() {
        let mut r = reader_from(&[0xFF, 0x00]);
        assert_eq!(r.read_u16().unwrap(), 255u16);
    }

    #[test]
    fn test_read_i32_negative_one() {
        let mut r = reader_from(&[0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(r.read_i32().unwrap(), -1i32);
    }

    #[test]
    fn test_read_u32() {
        let bytes = 1u32.to_le_bytes();
        let mut r = reader_from(&bytes);
        assert_eq!(r.read_u32().unwrap(), 1u32);
    }

    #[test]
    fn test_read_f32_one() {
        let bytes = 1.0f32.to_le_bytes();
        let mut r = reader_from(&bytes);
        assert!((r.read_f32().unwrap() - 1.0f32).abs() < 1e-7);
    }

    #[test]
    fn test_read_bool_false() {
        let mut r = reader_from(&[0]);
        assert!(!r.read_bool().unwrap());
    }

    #[test]
    fn test_read_bool_true_nonzero() {
        let mut r = reader_from(&[1]);
        assert!(r.read_bool().unwrap());
        let mut r = reader_from(&[0x42]);
        assert!(r.read_bool().unwrap());
    }

    #[test]
    fn test_read_bytes_n() {
        let mut r = reader_from(&[1, 2, 3, 4]);
        assert_eq!(r.read_bytes(3).unwrap(), vec![1u8, 2, 3]);
    }

    #[test]
    fn test_read_vec3() {
        let mut data = Vec::new();
        data.extend_from_slice(&1.0f32.to_le_bytes());
        data.extend_from_slice(&2.0f32.to_le_bytes());
        data.extend_from_slice(&3.0f32.to_le_bytes());
        let mut r = reader_from(&data);
        let v = r.read_vec3().unwrap();
        assert!((v.x - 1.0).abs() < 1e-7);
        assert!((v.y - 2.0).abs() < 1e-7);
        assert!((v.z - 3.0).abs() < 1e-7);
    }

    #[test]
    fn test_read_quat_normalized() {
        let mut data = Vec::new();
        data.extend_from_slice(&0.0f32.to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());
        data.extend_from_slice(&1.0f32.to_le_bytes());
        let mut r = reader_from(&data);
        let q = r.read_quat().unwrap();
        let len = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        assert!((len - 1.0).abs() < 1e-6, "quaternion not normalized: len={}", len);
        assert!((q.w - 1.0).abs() < 1e-6); // identity
    }

    #[test]
    fn test_read_string_sjis_fixed_null_terminated() {
        let data = b"Hello\0garbage";
        let mut r = reader_from(data);
        assert_eq!(r.read_string_sjis_fixed(13).unwrap(), "Hello");
    }

    #[test]
    fn test_read_string_sjis_fixed_no_null() {
        let mut r = reader_from(b"ABC");
        assert_eq!(r.read_string_sjis_fixed(3).unwrap(), "ABC");
    }

    #[test]
    fn test_read_dotnet_string_empty() {
        let mut r = reader_from(&[0x00]);
        assert_eq!(r.read_dotnet_string().unwrap(), "");
    }

    #[test]
    fn test_read_dotnet_string_ascii() {
        let mut data = vec![5u8];
        data.extend_from_slice(b"Hello");
        let mut r = reader_from(&data);
        assert_eq!(r.read_dotnet_string().unwrap(), "Hello");
    }

    #[test]
    fn test_read_7bit_int_one_byte() {
        let mut r = reader_from(&[0x05]);
        assert_eq!(r.read_7bit_encoded_int().unwrap(), 5);
    }

    #[test]
    fn test_read_7bit_int_two_bytes() {
        // 128 = [0x80, 0x01]
        let mut r = reader_from(&[0x80, 0x01]);
        assert_eq!(r.read_7bit_encoded_int().unwrap(), 128);
    }

    #[test]
    fn test_read_7bit_int_300() {
        // 300 = 0b100101100 → [0xAC, 0x02]
        let mut r = reader_from(&[0xAC, 0x02]);
        assert_eq!(r.read_7bit_encoded_int().unwrap(), 300);
    }

    #[test]
    fn test_read_eof_error() {
        let mut r = reader_from(&[]);
        assert!(r.read_u8().is_err());
    }

    #[test]
    fn test_seek_by() {
        let mut r = reader_from(&[1, 2, 3, 4]);
        r.seek_by(2).unwrap();
        assert_eq!(r.read_u8().unwrap(), 3);
    }
}
