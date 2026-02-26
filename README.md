# mmd2jsx

MikuMikuDance（MMD）のモーションデータを After Effects（AE）用の JSX スクリプトに変換する CLI ツールです。

遊太郎様作成の MMD2AE.EXE を参考にしました。

Inwerwm 様作成の MikuMikuMethods を参考にしました。

> **参考**:
>
> MikuMikuDance To After Effects 1.3 (C) 2010 Programmed by 遊太郎
>
> [MikuMikuMethods](https://github.com/Inwerwm/MikuMikuMethods) by Inwerwm

## 機能

### bone サブコマンド
- PMM ファイルからボーンのワールド座標を逐フレーム計算し、AE ヌルレイヤーのキーフレームとして出力
- FK（前向き運動学）・IK（逆運動学）・付与（通常付与・ローカル付与）を完全計算
- 外部親変形に対応
- ベジェ曲線補間による全フレーム生成または キーフレームのみ出力モードを選択可能
- 複数ボーンを一括出力

### camera サブコマンド
- **VMD / PMM** ファイルのカメラデータを AE カメラキーフレームに変換（拡張子で自動判別）
- PMM のボーン追従カメラに対応（追従モデル・ボーンが指定されている場合は FK/IK 計算後の位置に追従）
- ベジェ曲線補間による滑らかなフレーム生成（6 パラメータ独立補間）
- リニア区間の中間フレーム省略・値変化なしフレームの自動スキップによる出力最適化

## インストール

### ソースからビルド

```bash
cargo build --release
```

ビルド後、`target/release/mmd2jsx`（Windows では `mmd2jsx.exe`）が生成されます。

## 使い方

```
mmd2jsx <SUBCOMMAND> [OPTIONS]
```

### bone サブコマンド

```
mmd2jsx bone -i <PMM> (-m <モデル名> | --model-index <N>) -b <ボーン名>... [-o <JSX>]
```

#### オプション一覧

| オプション | 短縮形 | デフォルト | 説明 |
|-----------|--------|-----------|------|
| `--input <PMM>` | `-i` | 必須 | 入力 PMM ファイルパス |
| `--model <NAME>` | `-m` | | 対象モデル名（部分一致）。`--model-index` と排他 |
| `--model-index <N>` | | | `--list-models` の表示順インデックス。`--model` と排他 |
| `--bone <NAME>...` | `-b` | 必須 | 対象ボーン名（部分一致、複数指定可） |
| `--list-models` | | off | PMM 内のモデル一覧を表示して終了 |
| `--output <JSX>` | `-o` | 標準出力 | 出力 JSX ファイルパス |
| `--fps <N>` | | `30` | フレームレート |
| `--pmx <PMX>` | | PMM 埋め込みパス | PMX ファイルパスを手動指定 |
| `--keyframes-only` | | off | キーフレームと補間が必要なセグメントのみ出力 |
| `--width <N>` | | PMM ヘッダ値 | コンポジションの幅（px） |
| `--height <N>` | | PMM ヘッダ値 | コンポジションの高さ（px） |
| `--aspect <N>` | | `1.0` | ピクセル縦横比 |
| `--scale <N>` | | `20.0` | MMD → AE 座標スケール係数 |
| `--comp-name <NAME>` | | モデル名 | コンポジション名 |

#### 使用例

```bash
# モデル一覧を確認
mmd2jsx bone -i project.pmm --list-models

# 腰ボーンのワールド座標を JSX に出力
mmd2jsx bone -i project.pmm -m キャラ名 -b 腰 -o bone.jsx

# PMX を手動指定（PMM にパスが正しく記録されていない場合）
mmd2jsx bone -i project.pmm -m キャラ名 -b 腰 --pmx C:/path/to/model.pmx -o bone.jsx

# 複数ボーンを一括出力
mmd2jsx bone -i project.pmm --model-index 0 -b 頭 首 腰 -o bones.jsx

# キーフレームのみ出力（補間区間はスキップ）
mmd2jsx bone -i project.pmm -m キャラ名 -b 腰 --keyframes-only -o bone.jsx
```

---

### camera サブコマンド

```
mmd2jsx camera -i <VMD|PMM> [-o <JSX>]
```

入力ファイルは `.vmd` または `.pmm`（拡張子で自動判別）。

#### オプション一覧

| オプション | 短縮形 | デフォルト | 説明 |
|-----------|--------|-----------|------|
| `--input <FILE>` | `-i` | 必須 | 入力ファイルパス（.vmd または .pmm） |
| `--output <JSX>` | `-o` | 標準出力 | 出力 JSX ファイルパス |
| `--fps <N>` | `-f` | `30` | フレームレート |
| `--width <N>` | `-W` | VMD: 512、PMM: ヘッダ値 | コンポジションの幅（px） |
| `--height <N>` | `-H` | VMD: 384、PMM: ヘッダ値 | コンポジションの高さ（px） |
| `--pixel-aspect <N>` | `-p` | `1.0` | ピクセル縦横比 |
| `--comp-name <NAME>` | | `MMD CAMERA` | コンポジション名 |
| `--camera-name <NAME>` | | `MMD CAMERA` | カメラ名 |

#### 使用例

```bash
# VMD: 基本的な変換
mmd2jsx camera -i camera.vmd -o camera.jsx

# VMD: 1080p、60fps
mmd2jsx camera -i camera.vmd -o camera.jsx -W 1920 -H 1080 -f 60

# PMM: 幅・高さを PMM ヘッダから自動取得
mmd2jsx camera -i project.pmm -o camera.jsx

# PMM: 幅・高さを手動指定
mmd2jsx camera -i project.pmm -o camera.jsx -W 1920 -H 1080

# コンポジション名・カメラ名を指定
mmd2jsx camera -i camera.vmd -o camera.jsx --comp-name "Shot_01" --camera-name "Camera_01"
```

---

## After Effects での使い方

1. 変換で生成された `.jsx` ファイルを After Effects で読み込む
   - メニュー: **ファイル → スクリプト → スクリプトファイルを実行**
2. スクリプトを実行するとコンポジションとレイヤーが自動生成されます

### bone: 生成されるレイヤー構成

ボーンごとに 3D ヌルレイヤーが生成されます。

```
コンポジション（--comp-name で指定）
├── <ボーン名1>  ← 3D ヌルレイヤー（position にキーフレーム）
├── <ボーン名2>
└── <ボーン名N>
```

各フレームの `position` に `[AE_x, AE_y, AE_z]` がセットされます。

### camera: 生成されるレイヤー構成

```
コンポジション（--comp-name で指定）
├── MMD CAMERA CONTROL Y  ← Y 軸回転・位置制御（ヌルレイヤー）
│   └── MMD CAMERA CONTROL X  ← X 軸回転・距離制御（ヌルレイヤー）
│       └── <カメラ名>  ← カメラ本体（--camera-name で指定）
```

---

## 入力ファイルの制約

### VMD（camera サブコマンドのみ）
- カメラモーション VMD のみ対応（モデルモーション不可）
- モデル名が `カメラ・照明` であること

### PMM
- MikuMikuDance Ver.9.00（`Polygon Movie maker 0002`）フォーマットのみ対応
- bone サブコマンドの FK/IK 計算には PMX ファイルへのアクセスが必要（PMM に埋め込まれたパスを参照）

---

## ピクセル縦横比のプリセット

| 用途 | 値 |
|------|-----|
| 正方ピクセル | `1.0` |
| D1/DV NTSC | `0.91` |
| D1/DV NTSC ワイドスクリーン | `1.21` |
| D1/DV PAL | `1.09` |
| D1/DV PAL ワイドスクリーン | `1.46` |
| HDV 1080 / DVCPRO HD 720 | `1.33` |
| DVCPRO HD 1080 | `1.5` |
| アナモルフィック 2:1 | `2.0` |

---

## 実装の詳細

### bone: 変換式

スケール係数 `--scale`（デフォルト 20.0）、MMD 右手座標系（X:右, Y:上, Z:前）から AE（Y 軸反転）に変換します。

| AE プロパティ | 変換式 |
|---|---|
| position.x | `world_x × scale + Width/2` |
| position.y | `−world_y × scale + Height/2`（Y 反転） |
| position.z | `world_z × scale` |

### bone: FK/IK 計算フロー

1. **PMX 読み込み**: ボーン階層・IK 情報・付与情報（`format/pmx.rs`）
2. **PMM 読み込み**: 各フレームの移動・回転・IK 有効状態（`format/pmm.rs`）
3. **ベジェ補間**: 各フレーム間を 6 軸独立補間（`motion/interpolation.rs`）
4. **FK 計算**: `transform_order` 昇順でボーン変換を伝播
5. **付与適用**: 通常付与・ローカル付与（FK ループ内）
6. **IK 計算**: CCD-IK（角度制限・ループ制限対応）
7. **IK 後付与再計算**: IK リンク回転量も付与に反映（MMD 仕様 1.2）
8. **外部親変形**: PMM コンフィグフレームから外部親情報を読み込み適用

### camera: 変換式

スケール係数 20.0、MMD 右手座標系（X:右, Y:上, Z:前）から AE（Y 軸反転）に変換します。

| AE プロパティ | 変換式 |
|---|---|
| layNully.position.x | `location_x × 20 + Width/2` |
| layNully.position.y | `−location_y × 20 + Height/2`（Y 反転） |
| layNully.position.z | `location_z × 20` |
| layNully.yRotation | `−rotation_y × (180/π)`（Y 反転） |
| layNullx.anchorPoint.z | `−length × 20` |
| layNullx.xRotation | `rotation_x × (180/π)` |
| layNullx.zRotation | `rotation_z × (180/π)` |
| layCam.zoom | `(Height/2) / tan(fov/2 × π/180)` |

### camera: ボーン追従（PMM のみ）

PMM のカメラキーフレームにボーン追従設定（追従モデル・追従ボーン）が含まれる場合、対象ボーンの FK/IK 計算後のワールド座標をカメラ注視点位置に加算します。追従設定の on/off 切り替えはキーフレーム単位でステップ変化します。

### ベジェ曲線補間

VMD / PMM とも 6 パラメータ（X/Y/Z 移動、回転、距離、視野角）を独立して補間します。

**VMD** の `interpolation[24]` バイト構造:

| バイト位置 | パラメータ | レイアウト |
|-----------|-----------|-----------|
| [0–3]     | X 移動     | [ax, bx, ay, by]（0–127、127 で正規化） |
| [4–7]     | Y 移動     | 同上 |
| [8–11]    | Z 移動     | 同上 |
| [12–15]   | 回転       | 同上 |
| [16–19]   | 距離       | 同上 |
| [20–23]   | 視野角     | 同上 |

**PMM** の補間曲線は各パラメータ u8 × 4（値域 0–127）で格納、格納順は [ax, ay, bx, by]。

補間は CSS `cubic-bezier` と同じ 3 次ベジェ曲線（二分探索、f64 精度）で評価します。ボーン・カメラ共通の `motion/bezier.rs` モジュールで実装されています。

### 出力最適化（camera）

- **リニアスキップ**: 全パラメータの補間曲線がリニア（ax=ay かつ bx=by）な区間は中間フレームを生成しない
- **値変化スキップ**: 各プロパティが前フレームから変化しない場合はそのプロパティを出力しない（JSX 出力精度 6 桁で比較）
