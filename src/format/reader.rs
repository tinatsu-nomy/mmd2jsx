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
