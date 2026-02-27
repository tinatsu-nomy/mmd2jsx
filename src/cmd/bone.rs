// bone サブコマンド実装
// PMMファイルからボーン位置データを読み込み、FK/IK 計算後に After Effects 用 JSX を生成する。

use crate::cmd::cli::BoneArgs;
use crate::format::{pmm, pmx};
use crate::motion::interpolation;
use crate::jsx::bone as bone_jsx;

pub fn run(args: BoneArgs) -> Result<(), i32> {
    // PMMファイル読み込み
    let pmm_data = pmm::read_pmm(&args.input).map_err(|e| {
        eprintln!("Failed to load PMM file: {}", e);
        1
    })?;

    // render_order 昇順のインデックス表
    let sorted_order: Vec<usize> = {
        let mut v: Vec<(usize, u8)> = pmm_data.models.iter().enumerate()
            .map(|(i, m)| (i, m.render_order)).collect();
        v.sort_by_key(|&(_, ord)| ord);
        v.into_iter().map(|(i, _)| i).collect()
    };

    // --list-models
    if args.list_models {
        print_model_list(&pmm_data.models, &sorted_order);
        return Ok(());
    }

    // モデル検索
    let pmm_model_idx = if let Some(n) = args.model_index {
        if n >= sorted_order.len() {
            eprintln!("Model index {} is out of range (model count: {})", n, sorted_order.len());
            print_model_list(&pmm_data.models, &sorted_order);
            return Err(1);
        }
        sorted_order[n]
    } else if let Some(ref name) = args.model {
        let exact = pmm_data.models.iter().position(|m| &m.name == name);
        match exact.or_else(|| pmm_data.models.iter().position(|m| m.name.contains(name.as_str()))) {
            Some(idx) => idx,
            None => {
                eprintln!("Model '{}' not found", name);
                print_model_list(&pmm_data.models, &sorted_order);
                return Err(1);
            }
        }
    } else {
        eprintln!("Specify either --model or --model-index");
        print_model_list(&pmm_data.models, &sorted_order);
        return Err(1);
    };
    let pmm_model = &pmm_data.models[pmm_model_idx];

    // --bone 未指定
    if args.bone.is_empty() {
        eprintln!("Specify --bone");
        eprintln!("Available bones:");
        for b in &pmm_model.bones { eprintln!("  - {}", b.name); }
        return Err(1);
    }

    // ボーン名 → PMM インデックス解決
    let mut target_bone_indices: Vec<usize> = Vec::new();
    for bone_name in &args.bone {
        let exact = pmm_model.bones.iter().position(|b| &b.name == bone_name);
        match exact.or_else(|| pmm_model.bones.iter().position(|b| b.name.contains(bone_name.as_str()))) {
            Some(idx) => target_bone_indices.push(idx),
            None => {
                eprintln!("Bone '{}' not found", bone_name);
                eprintln!("Available bones:");
                for b in &pmm_model.bones { eprintln!("  - {}", b.name); }
                return Err(1);
            }
        }
    }

    // 全モデルの PMX ボーンリストを構築
    let all_pmx_bones: Vec<Option<Vec<pmx::PmxBone>>> = pmm_data.models.iter().enumerate()
        .map(|(idx, model)| {
            let pmx_path = if idx == pmm_model_idx {
                if let Some(p) = &args.pmx { p.clone() } else { std::path::PathBuf::from(&model.path) }
            } else {
                std::path::PathBuf::from(&model.path)
            };
            if !pmx_path.exists() {
                if idx == pmm_model_idx {
                    eprintln!("Warning: PMX file not found: {} — skipping FK/IK", pmx_path.display());
                }
                return None;
            }
            match pmx::read_pmx(&pmx_path) {
                Ok(m) => Some(m.bones),
                Err(e) => {
                    if idx == pmm_model_idx {
                        eprintln!("Warning: failed to load PMX file: {} — skipping FK/IK", e);
                    }
                    None
                }
            }
        })
        .collect();

    // ボーンフレームデータ構築
    let bone_names: Vec<String> = target_bone_indices.iter()
        .map(|&idx| pmm_model.bones[idx].name.clone()).collect();

    let all_frames: Vec<Vec<bone_jsx::FrameData>> = target_bone_indices.iter()
        .map(|&bone_idx| interpolation::build_frames(
            &pmm_data, pmm_model_idx, &all_pmx_bones, bone_idx, args.fps, !args.keyframes_only,
        ))
        .collect();

    for (bone_name, frames) in bone_names.iter().zip(all_frames.iter()) {
        eprintln!("Model: {}, Bone: {}, Frames: {}", pmm_model.name, bone_name, frames.len());
    }

    // JSX設定
    let width     = args.width.unwrap_or(pmm_data.output_width);
    let height    = args.height.unwrap_or(pmm_data.output_height);
    let comp_name = args.comp_name.unwrap_or_else(|| pmm_model.name.clone());
    let jsx_config = bone_jsx::JsxConfig {
        width, height, aspect: args.aspect, fps: args.fps,
        scale: args.scale, comp_name, bone_names,
    };

    // 出力
    bone_jsx::output_jsx(&all_frames, &jsx_config, args.output.as_deref())
        .map_err(|e| { eprintln!("Failed to write output: {}", e); 1 })?;

    Ok(())
}

pub(crate) fn print_model_list(models: &[pmm::PmmModel], sorted_order: &[usize]) {
    eprintln!("{:<6}  model name", "order");
    for (i, &arr_idx) in sorted_order.iter().enumerate() {
        let m = &models[arr_idx];
        eprintln!("{:<6}  {} (model_id={}, render_order={})", i, m.name, m.model_id, m.render_order);
    }
}
