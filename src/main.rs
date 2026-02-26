// mmd2jsx: MikuMikuDance (VMD/PMM) → After Effects JSX 変換ツール
// エントリポイント。サブコマンド（bone / camera）をディスパッチする。

mod format;
mod motion;
mod jsx;
mod cmd;

use clap::Parser;

fn main() {
    if let Err(code) = execute() {
        std::process::exit(code);
    }
}

fn execute() -> Result<(), i32> {
    let args = cmd::cli::Args::parse();
    match args.command {
        cmd::cli::Command::Bone(a)   => cmd::bone::run(a),
        cmd::cli::Command::Camera(a) => cmd::camera::run(a),
    }
}
