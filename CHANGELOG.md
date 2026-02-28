# Changelog

All notable changes to this project will be documented in this file.
Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [0.1.4] - 2026-02-28

### Changed

- 生成 JSX のコメント行の権利者表示を変更:
  `// Reference: (C) 2010 Programmed by 遊太郎`
  → `// Reference: MikuMikuDance To After Effects 1.3 (C) 2010 Programmed by 遊太郎`
  （bone・camera 両サブコマンドの出力に適用）
- `README.md` にプリビルドバイナリのダウンロードリンク（GitHub Releases）を追加

---

## [0.1.3] - 2026-02-28

### Changed (品質改善・リファクタリング)

**Clippy lints 有効化**
- `Cargo.toml` に `[lints.clippy]` セクションを追加
  - `clone_on_copy` / `redundant_clone` を `deny` に設定
  - `clippy::all` を `warn` として一括有効化

**パフォーマンス改善**
- `model_id_to_arr: HashMap<u8, usize>` を `build_frames_with_fk` で一度だけ計算し、
  内部関数（`build_extern_parent_mats`・`calc_frame_range_with_extern`）に参照渡しするよう変更
  （フレームごとに HashMap を再生成していた O(n) コストを削除）
- `get_ik_enabled_at` / `get_extern_parents_at` を O(n) 線形スキャンから
  `partition_point` による O(log n) 二分探索に変更
- `generate_jsx`（bone / camera 両方）に `String::with_capacity()` を追加し
  リアロケーションを削減

**コード品質**
- `unwrap()` を `expect("reason")` に置き換え（`interpolation.rs`・`transform.rs` 各 1 箇所）
- `&mut Vec<T>` 引数を `&mut [T]` に変更（`ptr_arg` lint 対応、`transform.rs` 3 箇所）
- `collapsible_if` — ネストした `if` を `&&` let 束縛でフラット化（6 箇所）
- `contains_key + insert` を `Entry` API（`entry().or_insert`）に統一
- `map_or(true, ...)` を `is_none_or(...)` に変更（`camera.rs` 6 箇所）
- `needless_range_loop` — インデックスループをイテレータに変更（`pmm.rs`）
- モジュールトップコメントを `///` から `//!` に修正（`reader.rs`）
- `items_after_test_module` を `#[expect]` で抑制（`pmx.rs`・`camera.rs`）
- `print_literal` — フォーマット文字列中の文字列リテラルを除去（`cmd/bone.rs`）
- `solve_ccd_ik` / `recompute_world_mats_from` に `#[expect(clippy::too_many_arguments)]` を追加

### Fixed (バグ修正)
- `let _path` の誤った命名を `let path` に修正（未使用変数を示す `_` プレフィックスの誤用）
- `ExternParentEntry` （`Copy` 型）への不要な `.clone()` を除去
- `bone_names.clone()` していた箇所を `into_iter()` によるムーブに変更

---

## [0.1.2] - 2026-02

### Added (外部親機能)
- PMM コンフィグフレームから外部親情報（`ExternParentEntry`）を読み込み、変形に反映
- `PmmModel` に `model_id: u8` フィールドを追加（`model_id_to_arr` ルックアップのため）
- `ConfigFrame.extern_parents` / `PmmModel.parentable_bone_indices` /
  `PmmModel.initial_extern_parents` を追加
- `build_extern_parent_mats` — PMX ボーンインデックス → 外部親ワールド行列マップの構築
- `compute_model_world_transforms_at` — 参照先モデルのワールド変換計算（再帰防止付き）
- `calc_world_mat` — 外部親あり時の変形式
  `world = extern_parent_mat × T(local_movs) × R(local_rots)` を実装

---

## [0.1.1] - 2026-01

### Added (カメラ機能)
- VMD カメラファイル（カメラ専用 VMD）の読み込み対応
- PMM カメラセクションの読み込み対応
- `output_jsx_from_vmd` / `output_jsx_from_pmm` — After Effects カメラ JSX 出力
- `camera` サブコマンドを追加（`--input <VMD|PMM>` で自動判別）
- `layNully`（Y 回転）/ `layNullx`（X 回転）/ `layCam`（カメラ本体）の 3 レイヤー構成
- ベジェ補間をカメラパラメータ（位置・回転・距離・FOV）各軸に適用

---

## [0.1.0] - 2025-12

### Added (初期リリース)
- PMM ファイルパーサー（モデルセクション：ボーン・IK・ConfigFrame）
- PMX ファイルパーサー（ボーン階層・IK チェーン・付与情報）
- `bone` サブコマンド（`--input <PMM> --model <名前> --bone <名前>` でボーン位置 JSX 出力）
- FK 変換（`transform_order` 昇順、親ボーンの回転・移動を子へ伝播）
- 付与回転 / 付与移動（通常付与・ローカル付与・多重付与対応）
- CCD-IK ソルバー（角度制限・ループ回数制限対応）
- IK 後の付与再計算（IK リンク回転量を付与量に反映）
- ベジェ補間（各軸独立・ニュートン法によるパラメータ解法）
- PMX なし時のフォールバック動作（movement をそのまま世界座標として使用）
- `--keyframes-only` / 全フレームモード切り替え
- `--list-models` によるモデル一覧表示
