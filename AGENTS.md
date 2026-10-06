# Agents

## 仓库概览

C. Dux — 38 键分体无线人体工学键盘的硬件设计仓库（ZMK 固件、supermini nRF52840、Choc v1 轴、无二极管可翻面 PCB）。

这不是软件项目，三条工具链共存：

- **Ergogen v4**：由 `ergogen/config.yaml` 生成 PCB
- **KiCad 8**：手工布线、出 gerber（JLC 下单）
- **build123d（Python）**：3D 打印垂直站立底座 `case/build123d/stand.py`

ZMK 固件配置在独立仓库 `cyfdecyf/zmk-config`（本地构建，不在本仓库）。

## 常用命令

### Ergogen（PCB 生成）

```sh
cd ergogen && ergogen .    # 也是 VS Code 默认构建任务（.vscode/tasks.json）
```

要求 Ergogen ≥ 4.1.0（KiCad 8 封装所需），输出到 `ergogen/output/`。封装 3D 模型路径依赖环境变量 `EG_INFUSED_KIM_3D_MODELS`、`KICAD8_3RD_PARTY`（见 config.yaml）。

### 底座模型（case/build123d，uv 管理）

```sh
cd case/build123d
uv sync                         # 首次；按 uv.lock 建 .venv（build123d 0.11.1）
uv run stand.py --part base     # 共享底座（打印一次，不分左右）
uv run stand.py --part upright  # 左立板，默认 20° 后仰；--side right 出右立板
uv run stand.py --part joint    # 配合试块（下整件订单前先打这个验证公差）
uv run stand.py --part upright --tilt 10,15,20,25,30 --step   # 批量角度 + STEP
uv run stand.py --show          # 只推送 OCP CAD Viewer 预览，不导出文件
```

输出到 `case/build123d/out/`（已 gitignore）。默认只导 STL（切片用），`--step` 才同时导 STEP（代打首选格式）。
完整参数表与设计依据见 `case/build123d/README.md`——改底座几何前先读它。

### Lint

```sh
cd case/build123d && ruff check stand.py    # 配置在 ruff.toml（行宽 90、单引号、py313）
```

没有测试；正确性验证靠 `--part joint`/`--part fit` 实物试件与运行时断言（见下）。

## 架构与数据流

### PCB 流水线

`ergogen/config.yaml` 是 PCB 的唯一真实来源：

1. `points.zones`（matrix 3x5 + 外侧小指加高键 + thumbfan 三键）——每键 `column_net` 直连 GPIO，另一侧统一 GND（无二极管设计）
2. `outlines.board` 多边形——板框
3. `pcbs.c-dux.footprints`——全部封装摆放（热插拔轴座、MCU、JST 电池座、电源/复位开关、EC11 编码器、tenting puck、4 个 M4 安装孔）

流程：`ergogen .`

- `ergogen/output/pcbs/c-dux.kicad_pcb`
- 拷入 `pcb/vX.Y/c-dux.kicad_pcb`
- KiCad 8 手工布线成 `c-dux-routed.kicad_pcb`
- fabrication toolkit 出 gerber。

`pcb/v1.0`、`pcb/v2.0` 是已定稿版本的存档（gerber + 图片）。

### 关键不变量：stand.py 与 PCB 的孔位同步

`stand.py` 每次运行都从 `pcb/v2.0/c-dux.kicad_pcb` 解析 4 个 M4 孔位与板框尺寸，并与源码顶部的 `HOLES_EXPECTED` 表断言（±0.05 mm）。
**PCB 改版导致孔位/板框变化时，脚本会拒绝出图**——这是防底座与板子失配的主动设计，不能绕过。
此时需人工核对新坐标后，同步更新 `HOLES_EXPECTED` / `PCB_W_EXPECTED` / `PCB_H_EXPECTED`。

坐标换算（安装朝向：小指加高键朝下、拇指簇朝上，两轴相对 KiCad 视图翻转）：`u = Y_max − Y_kicad`（横向）、`v = X_kicad − X_min`（高度）。

### 底座几何

两件式结构：共享底座（对称、不分左右、只打印一次）+ 按后仰角的立板（背板 + 12 柱 + 靴座），榫键 + 2×M4 从底面拧接，换后仰角只重打立板。
背板上 3 组 M4 柱位（平面内旋转 10/20/30°，`--set-angles` 可改）用于调整 home row 倾斜；
柱位布局由求解器按约束自动排布（柱间距 ≥13 mm、旋转后板角距底座顶面 ≥2 mm），加强肋位置亦自动避让柱位。
几何参数是 `stand.py` 顶部的模块常量。

### 封装来源

- `ergogen/footprints/ceoloide/` 是 **git submodule**（`cyfdecyf/ergogen-footprints`，用户自己的 fork）
  — 克隆需 `--recurse-submodules`；对它的改动在 submodule 内单独提交
- 本地自建封装：`ergogen/footprints/*.js`（电源/复位开关、tenting puck）与 `*.kicad_mod`
- 根目录 `a-dux.yaml`、`rae-dux.yaml` 是上游键盘（A. Dux / rae-dux）的参考配置，不参与构建

## 约定

- 提交信息用英文，带区域前缀，如 `case/build123d: ...`、`image: ...`
- 注释/文档语言跟随各文件现状：`stand.py` 用英文，`case/build123d/README.md` 用中文，根 `README.md` 用英文
- 根 README 的「Tips for using Ergogen with KiCAD」记录了非按键封装的定位工作流
  - Ergogen 任意位置生成 → KiCad 鼠标摆放 → 标尺量相对位置 → 回填 yaml

