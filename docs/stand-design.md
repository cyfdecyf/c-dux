# stand.py 代码设计文档

面向**接手 `case/build123d/stand.py` 开发**的工程师。设计动机、装配流程、
公差体系、打印与代打指南在 [../case/build123d/README.md](../case/build123d/README.md),
本文不重复——本文回答"代码为什么长这样、改哪里要小心、怎么验证"。

## 1. 系统总览

```
pcb/v2.0/c-dux.kicad_pcb ──parse_pcb──┐
                                     ▼
                      Stand(pcb_w, pcb_h, holes, outline, …)
                       ├─ _solve_sets   柱位布局求解
                       ├─ _auto_ribs    肋位求解
                       └─ build(part) ──► STL/STEP ──► out/
```

- 单文件实现,依赖仅 build123d(0.11.1,uv 管理)+ 标准库;
- 输入是 KiCad PCB 文件(Ergogen 生成),**孔位/板框不手抄**;
- 输出按 `--part` 分:base(共享底座)/ upright(按角度立板)/
  full(装配预览)/ fit(平板验证件)/ joint(配合试块);
- 文件名:`stand_<side>_upright_t<tilt>[_flange<W>].stl`。
  注意 **stem 只编码 tilt 不编码 set-angles**——同一 t30 文件名可能对应
  不同柱位旋转,混用多组 `--set-angles` 时要另行区分(已知限制)。

## 2. 三个坐标系(接手第一件事)

| 坐标系 | 定义 | 用在哪 |
|---|---|---|
| KiCad 系 | X 沿列方向(板高),Y 沿行方向(板宽);Y 朝下 | `parse_pcb` 输入 |
| 板局部系 | x 右、y 沿背板向上、z 朝前(柱子方向);背板厚 `plate_t`,轮廓 y ∈ [0, `plate_h`] | `plate_and_posts()` 全部几何 |
| 世界系 | 床面 = XY 平面,底座占 z ∈ [0, `base_t`],**用户在 −Y**;座面(接合面)z = `base_t` = 6 | 靴座/榫键/螺钉/法兰,以及导出的打印姿态 |

映射关系:

- KiCad → 站立系(安装朝向:小指加高键朝下、拇指簇朝上):
  `u = Y_max − Y_kicad`(横向)、`v = X_kicad − X_min`(离小指侧板边高度)。
- 站立系 → 板局部系(求解器输出):
  `x = u + shift − pcb_w/2`,
  `y = v + bottom_ext − keyboard_drop (+ lift)`(板底名义上比 PCB 底边
  低 `bottom_ext` = 6.5 mm,整组图案再沿板下移 `keyboard_drop` = 28 mm;
  lift 由高度约束解出,见 §5.1)。
- 板框采样:`parse_pcb` 除孔位外还返回 Edge.Cuts 实际轮廓的 ~1 mm
  采样点(站立系 (u, v);ergogen 已把弧打散成短线,端点 + 线性插值
  即可)。高度约束的"最低点"以这些采样点为准,不用包围盒角点——
  t30/−35 的真实最低点 (u,v) = (76.7, 12.7),比虚拟角点高 ~31 mm。
- 板局部系 → 世界系(`upright_solid` 的 `Pos(0,0,origin_z) * Rot(X=90−tilt)`):
  `y_w = ct·y_l − st·z_l`,`z_w = 6 + st·y_l + ct·z_l`
  (ct = cos tilt,st = sin tilt)。板顶向后仰 tilt,柱子垂直于板面。

## 3. 数据流与入口

`main()`(文件尾部):

1. 解析 `--side/--part/--tilt/--set-angles/--post-h/--attach/--flange/…`;
   校验:flange ∈ [0, 30],tilt/set-angles 非空;
2. `parse_pcb(args.pcb)` → `(pcb_w, pcb_h, holes, outline)`;
3. 每个 tilt 构造一个 `Stand`(base 只构造一次,与 tilt 无关);
4. `stand.build(part, side)`:right 侧 = `mirror(body, Plane.YZ)`;
5. `emit()`:导 STL,`--step` 加导 STEP,`--show` 只推送 OCP CAD
   Viewer(不导文件)。

## 4. 函数职责地图

### 模块级

| 函数 | 职责 | 关键点 |
|---|---|---|
| `parse_pcb` | 读 PCB 文件 → 板框 + 4 孔位 + 板框采样点 | 只认 Ergogen 导出的 `gr_line/gr_arc` 直边端点(圆角弧中点不参与取框);采样点逐段 ~1 mm 插值、站立系返回,供高度约束用;孔位与 `HOLES_EXPECTED` 逐项断言,超差 `sys.exit` |
| `joint_keys` | 两根榫键 | 30×8×3.2 @ x=±12, y 12..20;顶缘 45° 导入倒角 |
| `joint_screw_bores` | 底座 Ø4.5 通孔 | @(±58, 14.5) |
| `joint_screw_head_pockets` | 沉头孔 | Ø8 圆柱 + 平面台肩(1.75 mm 环形桥接;已实印装配验证,仅孔内少量拉丝——切片问题,见 README 打印建议) |
| `joint_nut_pockets` | 靴座螺母座 | Ø4.5 让位孔 + 六角沉孔(对边 7.4,深 3.4)+ 六角收口唇 6.9(<螺母 7.0,压入后常驻;与沉孔同取向,方形孔永远过不了六角螺母) |
| `joint_grooves` | 靴座底榫槽 | 键 + 0.4 mm 间隙,**贯穿切割**(盲槽顶曾是 8.8 mm 单边桥,改为贯穿) |
| `_convex_hull` / `_ccw` | 二维凸包 / 环绕向 | 收腰轮廓与凸台多边形用 |

### `Stand` 类

| 成员 | 职责 | 关键点 |
|---|---|---|
| `__init__` | 派生量计算 | `face_z = plate_t + post_h`(PCB 承托面,求解器高度约束用)、`plate_h`(高度法则,见 §6 不变量 6)、`plate_half`、`bulges`、`rib_x`、`haunch_h`、`boot_rear`(两遍构建派生,见 §6.3/不变量 4);`origin_z = base_t` |
| `_solve_sets` | 柱位布局求解 | 见 §5.1 |
| `_auto_ribs` | 肋位求解 | 见 §5.2 |
| `_cap_haunch` | 限制托肩楔高度 | 防止楔脊出现在柱背面螺丝过孔/螺母沉孔口正后方(沉孔口外扩 ~4.3 mm,故取柱心后 6.5 mm);下限 2 mm——再低只是软化棱角的薄棱,承载靠肋 |
| `_taper_outline` | 收腰轮廓 | 见 §5.3 |
| `plate_and_posts` | 背板 + 柱(板局部系) | 见 §6.1 |
| `base_solid` | 底座(世界系) | 见 §6.2 |
| `upright_solid` | 立板(世界系) | 见 §6.3 |
| `_wedge` | YZ 三角楔(托肩/肋共用) | 前缘埋入背板 2 mm、背缘近竖直 → 免支撑 |
| `_breakaway_flange` | `--flange` 可掰法兰 | 厚 0.6(2 层),绕靴座足迹外伸 W;在唇口倒角**之后**并入,互不干涉 |
| `build` | 按 part 分发;right = YZ 镜像 | |

### `main` / 工具

`summary()` 打印包围盒/体积/估重;`push_viewer()` 只在 `--show` 时推送且
吞掉连接噪音;`overhang_report.py` 与 `layer_float_audit.py` 是两个独立
的网格审计工具(见 §8)。

## 5. 核心算法

### 5.1 `_solve_sets` — 柱位布局

输入每个安装组的平面内旋转角(`--set-angles`),输出 4 孔在板局部系的坐标:

1. 以 4 孔质心为中心旋转图案;
2. `v_floor`:旋转后板料允许到达的最低 v——保证板背面最低材料点距底座
   顶面 ≥ `BASE_TOP_MARGIN`(2 mm,唯一高度约束)。约束对象是
   Edge.Cuts 采样轮廓逐角度取 min,不是包围盒角点(右下角 bbox 角处
   无板料);
3. 每组在 ±`SHIFT_SPAN`(45)内按 `SHIFT_GRID`(3)步进暴力枚举横向平移,
   过滤:`|x| > PLATE_HALF_BOUND − 8`(64)或任意两柱距
   < `MIN_POST_CENTER`(11);
4. 评分字典序:max(最小柱距, 封顶 20)→ 组间跨度最小 → 偏移居中;
5. 无解 → 提示减少组数或减小角度。

高度/下移合成:`lift = max(0, v_floor − v'_min + keyboard_drop)`,柱位
`y = v' + bottom_ext − keyboard_drop + lift`——整组图案沿板下移
`keyboard_drop` 28 mm(用户实测"键盘下移 3cm"),下移吃掉多少余量,
同一点料就需要多少额外抬升;t30/−35 下移正好用完 2 mm 余量
(lift 归零,最低材料点距底座顶面 ~0.9 mm),`--set-angles -20` 时
约束仍然 binds(lift ≈ 8.7,最低点恰在 8.0)。

当前默认 `TILT 30 / SET_ANGLES [-35]`:单组 4 柱,柱位 y =
6.7 / 52.3 / 77.3 / 112.8(H3 最低,螺母沉孔完整落在板上,喇叭口足环
越出板底缘 ~0.8 mm——已知边界情况);左侧柱沉入肋带后,左肋候选全部
撞上接合螺钉避让区,只解出右侧 1 根肋(x ≈ 45,脚本打 warning),
底带 x ∈ [−50, 64.25](+侧凸台承接外摆柱)。

### 5.2 `_auto_ribs` — 肋位

- 肋带:柱 y ∈ [−3, `rib_h`+6] 内的柱,横向避让 ≥12 mm(螺丝过孔在板背面
  出口不能被肋挡住);
- 避让榫键(|x| ≤ 12+15+`rib_t`/2+1)与接合螺钉(距 58 < 4.3+`rib_t`/2+1);
- 候选 x 在 ±(`plate_half`−12)内步进 2;从最右候选开始最远点采样,
  肋间距 ≥15;目标 `RIB_TARGET` = 5,**实际可能更少**(t30/−35 只放下
  2 根,<3 会打印 warning)——这是布局拥挤的正常结果,不是 bug。

### 5.3 `_taper_outline` — 收腰

底带以上(`y0 = rib_h/ct + taper_margin`)只保留“柱位包络凸包”:
柱心 ±(`post_d/2 + post_edge_margin`)的 keep-box 与段顶两角(即每侧
底带边缘,见不变量 7)做二维凸包。背板永不接触 PCB,所以肋带以上
只需要柱子和腹板。若 `y0 ≥ plate_h` 返回空(整板保留,不收腰)。

## 6. 实体构造顺序(布尔管线)

### 6.1 `plate_and_posts`(板局部系)

1. `RectangleRounded(plate_w, plate_h, 8)` 拉伸 `plate_t`;
2. 减收腰切割体(段上方 − 凸包);
3. 每侧封边切割:按 `__init__` 派生的每侧底带边缘(不变量 7)切掉
   `over = plate_half − |band_edge|`;**`over ≈ 0` 时必须跳过**(零宽
   Box 会令 OCP 抛异常——3f09adf 修复的坑);有凸台侧走 1/4 圆角肩的
   多边形切割路径;
4. 柱:`Cylinder(Ø10, post_h)` + 底部 45° 喇叭口(`Ø15→Ø10` × 1.5,自
   支撑);柱顶实心,直接承托 PCB(Ø4.5 孔留 2.75 mm 径向壁);
5. `--attach nut`:背面六角螺母沉孔,开口在背板**背面**、径向完全落在
   Ø10 柱足印内——z 0..1.2 收口唇(对边 6.9,六角取向)、z 1.2..4.6
   螺母位(对边 7.4 × 深 3.4);总深 4.6,留 1.4 mm 前壁由柱直接支撑,
   夹紧载荷路径 螺母 → 前壁 → 柱(全程受压,无打印桥接受力)+
   Ø4.5 螺丝通孔(背面直通到柱顶;穿过沉孔体积无妨,都是切割);螺钉
   M4×12(夹持 PCB 1.6 + 柱 3 + 壁 1.4 ≈ 6 mm + 螺母 3.4);
   `tap`:Ø3.4 自攻底孔,同样背面直通到柱顶。

### 6.2 `base_solid`(世界系)

底板 → 竖直边圆角 10 → 顶面周边圆角 2 → 6 个胶垫凹槽(卡缘 + 全深
平底,中部 1 mm 减胶槽把顶面拆成两个小桥;1 mm 胶垫高出底面 0.4)
→ 并榫键 → 减螺钉通孔与沉头孔。

### 6.3 `upright_solid`(世界系,顺序即打印姿态自上而下的层积)

1. `plate_and_posts()` 旋转到位;
2. **下前缘 45° 倒角**(`lip_chamfer`,锚在板座面角点 y=0):背板底缘面
   原本以 (90°−tilt) 掠过床面,是逐层前移的悬浮斜坡;倒角后每层前移
   ≤半根线宽。必须在靴座并集**之前**切;
3. 靴座 `Box(flush_w, self.boot_rear − boot_front, 8)`(z 6..14;
   `flush_w = min(plate_w, bx − 2·upright_inset)`,t30/−35 为 128.5,
   每侧比底座窄 2 mm 避开顶面圆角;接合螺钉处保留半宽 64.25,螺母座
   外侧壁 1.95 mm,薄但实心)。
   `boot_rear` 是**实例派生值**(模块常数 34 只是不下探的下限):
   `__init__` 先按硬下限(螺母座后壁 ≥1.5 mm、肋底 6 mm 后壁)建临时
   件,实测质心后把后沿伸到"质心投影距后沿 ≥8 mm",向上取偶数 mm
   重建一次——这是不变量 4 的落点,也是每条导出日志里 boot 行的来源;
4. 托肩楔(深 12,高度按 `_cap_haunch` 自动封顶 2–10)+ 肋楔(底深
   `rib_t+8`=20、高 `rib_h`=70、厚 `rib_t`=12,位置来自 `_auto_ribs`)
   ——都在靴座上、贴背板背面。(托肩 18→12、肋底 30→20:首版打印
   "底部偏臃肿"反馈后的收窄,强度按 4 层壁打印承担;托肩本就几乎全藏
   在靴座盒内,收窄不改体积,省料主要来自肋;t30/−35 图案下移后实例
   高度只剩下限 2 mm,左侧肋被螺钉避让区挤掉,只剩右肋)
5. `--flange` 时并上可掰法兰;
6. 减榫槽(贯穿)、螺母座;
7. 减 z<6 修剪盒(把低倾角下探到座面以下的柱喇叭口削平,底面落床)。

## 7. 常量分组

**调节旋钮**(改变造型/布局,改动后重跑审计即可):`plate_margin、
taper_margin、haunch_d、rib_*、RIB_TARGET、SHIFT_*、TILT、SET_ANGLES、
post_h(CLI)、flange_t、lip_chamfer、pad_*`。`keyboard_drop = 28` 特殊:
用户实测需求("键盘下移 3cm"收敛到 28),整组安装图案沿板下移;
上限受最低柱限制——喇叭口足环(半径 7.5)与背面螺母沉孔不得越出板底
缘,当前布局压线(柱心 y 6.7,足环越缘 ~0.8 mm,再加大必须先裁矮
喇叭口或减 drop)。`bottom_ext = 6.5` 只是名义板底延伸:实际板底缘
= `bottom_ext − keyboard_drop + lift`,已基本贴住键盘最低材料点。
`boot_rear` 特殊:模块常数只是默认下限,实例值由 `__init__` 两遍构建
按质心自动后伸(见 §6.3),改它请改派生规则里的余量(8 mm)而不是常数。

**载荷/工艺设计值**(改前先读 README 对应章节,并与实物验证挂钩):
`joint_clear = 0.4`(FDM 公差基线)、`boot_nut_clear = 3.6`(抗蠕变承压)、
`nut_mouth_af = 6.9`(对螺母 7.0 的六角过盈收口,与沉孔同取向,靴座与
背面 PCB 沉孔共用同一 trick)、`nut_pocket_depth = 4.6` +
`nut_mouth_h = 1.2`(背面 PCB 螺母沉孔:唇 + 螺母位,留 1.4 mm 前壁由
柱直接支撑)、`post_d = 10`(实心柱,Ø4.5 孔留 2.75 mm 径向壁)+
`post_edge_margin = 5`(keep = 10,仍盖住 Ø8.6 螺母座 + 唇)、
`joint_screw_d = 4.5`(实印
基准)、`MIN_POST_CENTER = 11`、`BASE_TOP_MARGIN = 2`、`key_h = 3.2 <
boot_nut_clear = 3.6`(立板承压落在座面而非键顶)、`base_t = 6`;胶垫
凹槽 `pad_recess = pad_t − pad_protrude = 0.6`(按 1 mm 胶垫留 0.4
凸出,垫独占桌面接触)、`pad_slot_w/d = 1`(减胶槽,两处 3 mm 粘接
平台断开,防中心盘顶面孤岛)。

## 8. 不变量清单(改代码前必读)

1. **PCB 防呆**:`HOLES_EXPECTED` 断言永不绕过;PCB 改版后人工核对坐标
   (换算见 §2)再更新表;
2. **接合分工**:座面受压、榫键抗剪定位、2×M4 抗拉拔——挪键/螺钉前先
   重新推导;螺丝从底座底面向上拧(立板后仰后上方没有螺丝刀空间);
3. **免支撑**:任何几何改动后重跑 §8 的两个审计;新悬空面按"每层前移
   ≤ 半根线宽"原则处理(倒角锚点就是这一规则的实例);
4. **可自由站立**:质心平面投影必须落在靴座足迹内、距后沿 ≥8 mm。
   已数据驱动:`__init__` 两遍构建自动后伸 `boot_rear`(t30/−35 实例
   后沿 38,实测质心 y 29.1,余量 8.9 mm);
5. **公差基线 0.4 mm**:活动/配合面全部按此取;**切片端 X-Y 孔补偿保持
   0**,否则收口唇 6.9 > 螺母 7.0,常驻螺母失效;
6. **背板不接触 PCB**:`post_h` 是间隙;背板只升到最高柱顶 + `top_rim`;
7. **底带每侧收窄 + 2 mm 内缩**:已打印的底座(132.5)是宽度基准;每侧
   底带取“该侧最外柱(+2)与肋(+1)的包络,偶数 mm 取整”与
   `bx/2 − 2` 的较小值——内缩 2 mm 避开底座顶面圆角,齐边会悬在圆角
   上;柱位越界的一侧保持带在界上、走凸台(quarter-round)承接,不加宽
   整板;靴座仍保留接合螺钉所需宽度(半宽 `bx/2 − 2` = 64.25);
8. 座面 z = 6 是唯一接合基准,所有连接特征用世界坐标定义、与倾角无关。

## 9. 验证工具与流程

| 工具 | 输入 | 检查什么 |
|---|---|---|
| `overhang_report.py` | STL | 面级:按坡度分桶的下行面积 + <45° 连通簇定位(线宽无关) |
| `layer_float_audit.py` | STL | 层级:0.2 mm 逐层切片差分,真实悬空面积/最差层/"空中起印"孤岛(真失败风险) |
| `--part joint` | — | 榫键/榫槽/螺母座实物配合,任何几何改动后先打 |

两者都以导出姿态(STL 即打印姿态)、0.2 mm 层、~0.25 mm 支撑带宽为假设。

**当前基线**(t30/−35 左立板,Ø10 柱 + 收窄底带后):悬空合计
0.29 cm²(合 0.01 cm³),最差单层
1.2 mm²(柱腹线),无孤岛;质心 (6.8, 29.1, 52.6)。回归时数字应同量级。

```sh
uv run --with trimesh --with shapely --with numpy --with networkx \
  --with rtree python layer_float_audit.py out/stand_left_upright_t30.stl
uv run --with numpy python overhang_report.py out/stand_left_upright_t30.stl
```

完整实物验证清单见 README"首版验证清单"。

## 10. 已知限制与扩展点

- stem 不编码 set-angles(§1);多角度组合并存时需自行区分;
- 柱位外摆时肋会少于 `RIB_TARGET`(设计如此);靴座半宽不足
  (`bx/2 − 2 < 58+5`)直接退出(--set-angles 太散);
- 有效倾角范围 10–40°(受接合螺钉 y=14.5 落在靴座台上约束);
- 腹板减重孔已整体移除(历史见提交 b187149;站立姿态下孔顶是 ~Ø52 桥,
  牺牲的省料由收腰承担);
- `parse_pcb` 假设 Ergogen 导出的 KiCad 文本格式(gr_line Edge.Cuts、
  `ceoloide:mounting_hole_plated` 封装名);KiCad 版本变化导致格式变化时
  这里最先坏。

## 11. 约定

- 提交信息英文,前缀 `case/build123d:`;文档类用 `docs:`;
- `stand.py` 注释英文,README 与本目录文档中文;
- 几何变更的提交应附审计数字(悬空面积/质心/体积),例见 git log。
