// CLI 引数定義
// clap derive マクロで Args / BoneArgs / CameraArgs を定義する。

use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "mmd2jsx")]
#[command(about = "Convert MikuMikuDance motion data (VMD/PMM) to After Effects JSX scripts")]
pub struct Args {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand, Debug)]
pub enum Command {
    /// Extract bone positions per frame and generate After Effects JSX
    Bone(BoneArgs),
    /// Convert camera data to After Effects JSX (VMD/PMM supported)
    Camera(CameraArgs),
}

// ─────────────────────────────────────────────
// bone サブコマンド引数
// ─────────────────────────────────────────────

#[derive(Parser, Debug)]
pub struct BoneArgs {
    /// Path to input PMM file
    #[arg(short, long)]
    pub input: PathBuf,

    /// Target model name (partial match). Exclusive with --model-index
    #[arg(short, long, conflicts_with = "model_index")]
    pub model: Option<String>,

    /// Target model index in display order as shown by --list-models. Exclusive with --model
    #[arg(long, conflicts_with = "model")]
    pub model_index: Option<usize>,

    /// Target bone name(s) (partial match, multiple allowed)
    #[arg(short, long, num_args(1..))]
    pub bone: Vec<String>,

    /// List all models in PMM in display order and exit
    #[arg(long)]
    pub list_models: bool,

    /// Output file path (default: stdout)
    #[arg(short, long)]
    pub output: Option<PathBuf>,

    /// MMD frame rate (default: 30)
    #[arg(long, default_value = "30")]
    pub fps: u32,

    /// Manually specify PMX file path (default: path embedded in PMM)
    #[arg(long)]
    pub pmx: Option<PathBuf>,

    /// Output only keyframes and segments requiring interpolation (default: all frames)
    #[arg(long)]
    pub keyframes_only: bool,

    /// Composition width (default: from PMM header)
    #[arg(long)]
    pub width: Option<i32>,

    /// Composition height (default: from PMM header)
    #[arg(long)]
    pub height: Option<i32>,

    /// Pixel aspect ratio (default: 1.0)
    #[arg(long, default_value = "1.0")]
    pub aspect: f64,

    /// Scale factor from MMD to AE coordinates (default: 20.0)
    #[arg(long, default_value = "20.0")]
    pub scale: f64,

    /// Composition name (default: model name)
    #[arg(long)]
    pub comp_name: Option<String>,
}

// ─────────────────────────────────────────────
// camera サブコマンド引数
// ─────────────────────────────────────────────

#[derive(Parser, Debug)]
pub struct CameraArgs {
    /// Input file path (.vmd or .pmm, auto-detected by extension)
    #[arg(short, long)]
    pub input: PathBuf,

    /// Output JSX file path (default: stdout)
    #[arg(short, long)]
    pub output: Option<PathBuf>,

    /// Frame rate (default: 30)
    #[arg(short, long, default_value = "30")]
    pub fps: u32,

    /// Composition width (VMD default: 512, PMM default: from PMM header)
    #[arg(short = 'W', long)]
    pub width: Option<u32>,

    /// Composition height (VMD default: 384, PMM default: from PMM header)
    #[arg(short = 'H', long)]
    pub height: Option<u32>,

    /// Pixel aspect ratio (default: 1.0)
    #[arg(short, long, default_value = "1.0")]
    pub pixel_aspect: f64,

    /// Composition name (default: "MMD CAMERA")
    #[arg(long, default_value = "MMD CAMERA")]
    pub comp_name: String,

    /// Camera layer name (default: "MMD CAMERA")
    #[arg(long, default_value = "MMD CAMERA")]
    pub camera_name: String,
}
