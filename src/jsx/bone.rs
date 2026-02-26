// ボーン位置 JSX 出力モジュール
// フレームデータを After Effects の null object キーフレームとして JSX スクリプトに出力する。

use std::io;
use std::path::Path;

/// 出力するフレームデータ
#[derive(Debug, Clone)]
pub struct FrameData {
    /// MMDフレーム番号
    pub frame: i32,

    /// ボーンのワールド座標 (MMD座標系: 右手系・Y上・Z前)
    pub world_x: f32,
    pub world_y: f32,
    pub world_z: f32,
}

/// ボーンJSX出力の設定
pub struct JsxConfig {
    pub width: i32,
    pub height: i32,
    pub aspect: f64,
    pub fps: u32,
    pub scale: f64,
    pub comp_name: String,
    pub bone_names: Vec<String>,
}

fn generate_jsx(all_frames: &[Vec<FrameData>], config: &JsxConfig) -> String {
    let max_frame = all_frames
        .iter()
        .flat_map(|frames| frames.iter().map(|f| f.frame))
        .max()
        .unwrap_or(0);
    let duration = max_frame as f64 / config.fps as f64 + 1.0 / config.fps as f64;

    let half_w = config.width as f64 / 2.0;
    let half_h = config.height as f64 / 2.0;
    let scale = config.scale;

    let mut s = String::new();

    s.push_str("//=================================================================\n");
    s.push_str("// MikuMikuDance To After Effects (Bone)\n");
    s.push_str("// Reference: (C) 2010 Programmed by 遊太郎\n");
    s.push_str("//=================================================================\n");
    s.push('\n');
    s.push_str("//- Composition Settings ------------------------------------------\n");
    s.push_str(&format!("var Width       = {};\n", config.width));
    s.push_str(&format!("var Height      = {};\n", config.height));
    s.push_str(&format!("var AspectRatio = {:.6};\n", config.aspect));
    s.push_str(&format!("var Duration    = {:.6};\n", duration));
    s.push_str(&format!("var FPS         = {};\n", config.fps));
    s.push('\n');

    s.push_str("//- Add Layers ----------------------------------------------------\n");
    s.push_str(&format!(
        "var newComp  = app.project.items.addComp( \"{}\", Width, Height, AspectRatio, Duration, FPS );\n",
        config.comp_name
    ));
    s.push_str("var layNull  = [];\n");
    for (i, bone_name) in config.bone_names.iter().enumerate() {
        s.push_str(&format!("layNull[{}] = newComp.layers.addNull();\n", i));
        s.push_str(&format!("layNull[{}].name        = \"{}\";\n", i, bone_name));
        s.push_str(&format!("layNull[{}].threeDLayer = true;\n", i));
        s.push_str(&format!(
            "layNull[{}].anchorPoint.setValue( [ {:.6}, {:.6}, {:.6} ] );\n",
            i, 0.0_f64, 0.0_f64, 0.0_f64
        ));
        s.push_str(&format!(
            "layNull[{}].position.setValue( [ {:.6}, {:.6}, {:.6} ] );\n",
            i, half_w, half_h, 0.0_f64
        ));
    }

    s.push('\n');
    s.push_str("//- Add Frames ----------------------------------------------------\n");
    for (bone_idx, (bone_name, frames)) in config.bone_names.iter().zip(all_frames.iter()).enumerate() {
        for (n, fd) in frames.iter().enumerate() {
            let time = fd.frame as f64 / config.fps as f64;
            let ae_x = fd.world_x as f64 * scale + half_w;
            let ae_y = -fd.world_y as f64 * scale + half_h;
            let ae_z = fd.world_z as f64 * scale;
            s.push('\n');
            s.push_str(&format!("//- frame ({}) -\n", bone_name));
            s.push_str(&format!(
                "layNull[{}].position.setValueAtTime( {:.6}, [ {:.6}, {:.6}, {:.6} ] );\n",
                bone_idx, time, ae_x, ae_y, ae_z
            ));
            s.push_str(&format!(
                "layNull[{}].position.setSpatialAutoBezierAtKey( {}, false );\n",
                bone_idx, n + 1
            ));
        }
    }

    s
}

/// JSX を path に書き込む（None の場合は標準出力）
pub(crate) fn emit_jsx(content: String, path: Option<&Path>) -> io::Result<()> {
    match path {
        Some(p) => std::fs::write(p, content),
        None    => { print!("{}", content); Ok(()) },
    }
}

pub fn output_jsx(all_frames: &[Vec<FrameData>], config: &JsxConfig, path: Option<&Path>) -> io::Result<()> {
    emit_jsx(generate_jsx(all_frames, config), path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config(
        width: i32, height: i32, fps: u32, scale: f64,
        bone_names: Vec<String>, comp_name: &str,
    ) -> JsxConfig {
        JsxConfig {
            width, height, aspect: 1.0, fps, scale,
            comp_name: comp_name.to_string(),
            bone_names,
        }
    }

    fn single_frame(x: f32, y: f32, z: f32) -> Vec<Vec<FrameData>> {
        vec![vec![FrameData { frame: 0, world_x: x, world_y: y, world_z: z }]]
    }

    #[test]
    fn test_generate_jsx_contains_header() {
        let config = make_config(1920, 1080, 30, 20.0, vec!["Bone".to_string()], "Comp");
        let jsx = generate_jsx(&single_frame(0.0, 0.0, 0.0), &config);
        assert!(jsx.contains("MikuMikuDance To After Effects (Bone)"));
    }

    #[test]
    fn test_generate_jsx_dimensions() {
        let config = make_config(1920, 1080, 30, 20.0, vec!["Bone".to_string()], "Comp");
        let jsx = generate_jsx(&single_frame(0.0, 0.0, 0.0), &config);
        assert!(jsx.contains("var Width       = 1920;"));
        assert!(jsx.contains("var Height      = 1080;"));
        assert!(jsx.contains("var FPS         = 30;"));
    }

    #[test]
    fn test_generate_jsx_comp_name() {
        let config = make_config(1920, 1080, 30, 20.0, vec!["Bone".to_string()], "TestComp");
        let jsx = generate_jsx(&single_frame(0.0, 0.0, 0.0), &config);
        assert!(jsx.contains("\"TestComp\""));
    }

    #[test]
    fn test_generate_jsx_bone_name_in_layer() {
        let config = make_config(1920, 1080, 30, 20.0, vec!["RightArm".to_string()], "Comp");
        let jsx = generate_jsx(&single_frame(0.0, 0.0, 0.0), &config);
        assert!(jsx.contains("\"RightArm\""));
    }

    #[test]
    fn test_generate_jsx_coordinate_transform() {
        // x=1, y=2, z=3, scale=20, 1920x1080
        // AE X = 1*20 + 960 = 980
        // AE Y = -2*20 + 540 = 500
        // AE Z = 3*20 = 60
        let config = make_config(1920, 1080, 30, 20.0, vec!["Bone".to_string()], "Comp");
        let jsx = generate_jsx(&single_frame(1.0, 2.0, 3.0), &config);
        assert!(jsx.contains("980.000000"), "AE X not found in JSX:\n{}", jsx);
        assert!(jsx.contains("500.000000"), "AE Y not found in JSX:\n{}", jsx);
        assert!(jsx.contains("60.000000"),  "AE Z not found in JSX:\n{}", jsx);
    }

    #[test]
    fn test_generate_jsx_time_calculation() {
        // frame=30, fps=30 → time = 1.000000
        let config = make_config(1920, 1080, 30, 20.0, vec!["Bone".to_string()], "Comp");
        let frames = vec![vec![FrameData { frame: 30, world_x: 0.0, world_y: 0.0, world_z: 0.0 }]];
        let jsx = generate_jsx(&frames, &config);
        assert!(jsx.contains("1.000000"), "time not found in JSX");
    }

    #[test]
    fn test_generate_jsx_duration() {
        // max_frame=60, fps=30 → duration = 60/30 + 1/30 = 2.033333...
        let config = make_config(1920, 1080, 30, 20.0, vec!["Bone".to_string()], "Comp");
        let frames = vec![vec![FrameData { frame: 60, world_x: 0.0, world_y: 0.0, world_z: 0.0 }]];
        let jsx = generate_jsx(&frames, &config);
        assert!(jsx.contains("2.033333"), "duration not found in JSX:\n{}", jsx);
    }

    #[test]
    fn test_generate_jsx_multi_bone() {
        let config = make_config(
            1920, 1080, 30, 20.0,
            vec!["Bone0".to_string(), "Bone1".to_string()], "Comp",
        );
        let frames = vec![
            vec![FrameData { frame: 0, world_x: 0.0, world_y: 0.0, world_z: 0.0 }],
            vec![FrameData { frame: 0, world_x: 0.0, world_y: 0.0, world_z: 0.0 }],
        ];
        let jsx = generate_jsx(&frames, &config);
        assert!(jsx.contains("layNull[0]"));
        assert!(jsx.contains("layNull[1]"));
    }
}
