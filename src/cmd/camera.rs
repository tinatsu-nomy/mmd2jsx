// camera サブコマンド実装
// VMD または PMM ファイルからカメラデータを読み込み、After Effects 用 JSX を生成する。

use crate::cmd::cli::CameraArgs;
use crate::format::{pmm, pmx, vmd};
use crate::jsx::camera;

pub fn run(args: CameraArgs) -> Result<(), i32> {
    let ext = args.input.extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();

    match ext.as_str() {
        "vmd" => run_from_vmd(args),
        "pmm" => run_from_pmm(args),
        other => {
            eprintln!("Error: unsupported extension '.{}' (expected .vmd or .pmm)", other);
            Err(1)
        }
    }
}

fn run_from_vmd(args: CameraArgs) -> Result<(), i32> {
    let width  = args.width.unwrap_or(512);
    let height = args.height.unwrap_or(384);

    eprintln!("Loading VMD file: {}", args.input.display());
    let vmd_data = vmd::read_vmd(&args.input).map_err(|e| {
        eprintln!("Error: {}", e);
        1
    })?;

    eprintln!("✓ VMD file loaded");
    eprintln!("  Model name: {}", vmd_data.model_name);
    eprintln!("  Camera keyframes: {}", vmd_data.cameras.len());

    let config = camera::CameraJsxConfig {
        width, height, pixel_aspect: args.pixel_aspect,
        fps: args.fps, comp_name: args.comp_name, camera_name: args.camera_name,
    };

    camera::output_jsx_from_vmd(&vmd_data.cameras, &config, args.output.as_deref())
        .map_err(|e| { eprintln!("Error: failed to write JSX file: {}", e); 1 })?;

    if let Some(path) = &args.output {
        eprintln!("✓ Wrote JSX file: {}", path.display());
    }
    Ok(())
}

fn run_from_pmm(args: CameraArgs) -> Result<(), i32> {
    eprintln!("Loading PMM file: {}", args.input.display());
    let pmm_data = pmm::read_pmm(&args.input).map_err(|e| {
        eprintln!("Error: {}", e);
        1
    })?;

    eprintln!("✓ PMM file loaded");
    eprintln!("  Camera keyframes: {}", pmm_data.cameras.len());

    // 追従ボーン計算のために全モデルのPMXを読み込む
    let has_follow = pmm_data.cameras.iter().any(|c| c.follow_model >= 0);
    let all_pmx_bones: Vec<Option<Vec<pmx::PmxBone>>> = if has_follow {
        pmm_data.models.iter().map(|model| {
            let pmx_path = std::path::PathBuf::from(&model.path);
            if !pmx_path.exists() { return None; }
            match pmx::read_pmx(&pmx_path) {
                Ok(m) => Some(m.bones),
                Err(_) => None,
            }
        }).collect()
    } else {
        vec![None; pmm_data.models.len()]
    };

    if has_follow {
        let loaded = all_pmx_bones.iter().filter(|b| b.is_some()).count();
        eprintln!("  Follow bone enabled: PMX loaded {}/{} models", loaded, pmm_data.models.len());
    }

    let width  = args.width.unwrap_or(pmm_data.output_width as u32);
    let height = args.height.unwrap_or(pmm_data.output_height as u32);

    let config = camera::CameraJsxConfig {
        width, height, pixel_aspect: args.pixel_aspect,
        fps: args.fps, comp_name: args.comp_name, camera_name: args.camera_name,
    };

    camera::output_jsx_from_pmm(&pmm_data.cameras, &pmm_data, &all_pmx_bones, &config, args.output.as_deref())
        .map_err(|e| { eprintln!("Error: failed to write JSX file: {}", e); 1 })?;

    if let Some(path) = &args.output {
        eprintln!("✓ Wrote JSX file: {}", path.display());
    }
    Ok(())
}
