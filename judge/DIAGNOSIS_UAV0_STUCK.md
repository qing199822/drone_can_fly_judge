# 0 号无人机卡在 region1 边界 - 根因诊断

> 数据源：`logs/referee_round/round_20260921_212904/uav_0_{pose,diag}.log`
> 时间：2026-09-23
> 状态：已坐实，等用户决策下一步

## TL;DR

**0 号没有卡在西南角，它卡在 region1 的东边界（x_world≈-0.66），距离 region 中心 (-25,-25) 还有 24m，离 ROAD 目标 (-35,-35) 还有 34m。它从未到达 ROAD 目标，整轮 2745 个 diag 样本 99.4% 都在 mode=1（ROAD）阶段，从来没进 PATROL。**

**根因**：0 号起飞后朝 **x_world 增加方向**（实际朝向 `yaw=0°`，正东）飞，飞到 region_xmax_=0 边界附近 1m 内，被 `enforceRegionBounds` 的硬保护持续清零前向速度 → 永久卡在边界附近。

**为什么 hover-skip 没救**：hover-skip / stall / advance 三套机制都只在 `mode_ == PATROL` 下触发，0 号整轮都没进 PATROL，所以 5s/20s/2s 任何一个跳过机制都没机会运行。

## 关键证据

### 1. 0 号从未进 PATROL

`uav_0_diag.log` 2762 个样本里 mode 分布：

| mode | 含义 | 样本数 | 占比 |
|---|---|---|---|
| 0 | TAKEOFF | 17 | 0.6% |
| 1 | ROAD | 2745 | 99.4% |
| 2 | PATROL | 0 | 0% |
| 3 | TRANSIT_TO_REGION | 0 | 0% |

0 号整轮都在 ROAD，**根本没进 PATROL**，因此 hover-skip / stall / advance 三个跳过机制全部没机会运行。

### 2. 0 号实际位置 vs region 边界

UAV0 region 定义（`area_mission_node.cpp:434-435`）：
- `region = [-50, 0] × [-50, 0]`
- ROAD 目标 `(-35, -35)`，spawn `(-17, -3)`

末段 200 个 pose 样本的世界坐标范围：

| 坐标轴 | 最小 | 最大 | 距边界 |
|---|---|---|---|
| x_world | -0.75 | -0.64 | **0.66m 超出 region_xmax_=0** |
| y_world | -6.37 | -4.74 | 远在 ymin=-50 内部 |

末段最后 10 个 pose（每隔 20 个抽一个）：
```
world x = -0.754 / -0.725 / -0.709 / -0.692 / -0.686 / -0.675 / -0.662 / -0.650 / -0.644 / -0.648
over_xmax = 0.66 ~ 0.75m
全在 enforceRegionBounds margin=1.5m 范围内
```

### 3. 0 号为什么朝东飞

末段 0 号实际航向：

| 时间 | 位置 (world) | yaw |
|---|---|---|
| 起飞 (t=24s) | (-16.86, -3.14) | -2.0°（朝北微偏） |
| t=49s 首次到 x>-1 | (-0.99, -1.71) | -33.5° |
| 1/8 (t≈85s) | (1.00, -3.10) | -43.7° |
| 1/4 (t≈123s) | (-0.42, -6.79) | +26.0° |
| 1/2 (t≈192s) | (-1.80, -7.03) | -69.3° |
| 3/4 (t≈260s) | (-0.79, -7.34) | +4.2° |
| 末段-200 | (-0.75, -6.37) | +2.2° |
| 末段 | (-0.67, -4.74) | **-0.8°（几乎正东）** |

到 ROAD 目标 `(-35,-35)` 的理论方位角一直是 **-119° ~ -141°（朝西南方向）**，但 0 号末段 yaw 在 **0°附近（正东）**，与目标方位偏差 **+130°以上**。

也就是说，0 号没在朝 ROAD 目标飞——它在区域内部反复转圈，朝向最终稳定在正东方向（被边界挡住后无法再往东走）。

### 4. 末段 diag 状态

最后 100 个 diag 样本统计：

| 字段 | 值 |
|---|---|
| `mode=1`（ROAD） | 100% |
| `stop=7`（区域边界） | 100% |
| `region_out=0.0` | 100%（diag 字段计算的是 region_outside_dist 而不是世界坐标） |
| `v_mppi` | 1.50（MPPI 想要前进 1.5 m/s） |
| `v_pub` | ≤0.03（前向被清零） |
| `dist`（距 ROAD 目标） | 45.52 → 45.77（50+ 周期只漂移 0.25m） |
| `road_valid=1` | 100% |
| `region_clamp` | 200（触顶） |
| `region_return` | 0 / 1（不停切换） |

`region_clamp=200` 触顶说明 `region_clamp_killed_` 持续为 true，意味着 `enforceRegionBounds` 把 `forward_before≥0.3` 的前向速度清成了 `|cmd.linear.x|<0.1`。`region_return` 在 0/1 间切换说明 `region_outside_dist_` 在 0.5~2.0m 之间来回（因为无人机在边界 1.5m margin 与 2.0m 阈值之间抖动）。

### 5. enforceRegionBounds 直接命中

```cpp
// area_mission_node.cpp:1065-1071
const double margin = 1.5;
if (wx <= region_xmin_ + margin && vx < 0.0) vx = 0.0;  // 西边界
if (wx >= region_xmax_ - margin && vx > 0.0) vx = 0.0;  // 东边界 ← 这里命中
if (wy <= region_ymin_ + margin && vy < 0.0) vy = 0.0;  // 南边界
if (wy >= region_ymax_ - margin && vy > 0.0) vy = 0.0;  // 北边界
```

0 号末段 `wx = -0.66`，`region_xmax_ - margin = 0 - 1.5 = -1.5`，因此 `-0.66 >= -1.5` 成立。yaw≈0° 时 `vx = cos(0°) × 1.5 = +1.5 > 0` → vx 被清零。

但这个条件的"反面"逻辑也成立：0 号在 y 方向（`-50 ≤ -4.74 ≤ 0`）完全在区域内，南北方向都没有触发。所以问题不在 vy，而在 vx。

## 真实根因

0 号卡在 region1 **东边界（x_world≈-0.66）**，不是西南角。卡住不是因为"边界参数没生效"，而是因为：

1. **enforceRegionBounds 写死 margin=1.5m + 只清单一世界轴速度**（vy 未被清零，但 vx 被清零 → 无人机朝正东飞被挡、vy 还能动 → 沿边界 y 方向来回滑动）；
2. **0 号实际朝东飞而不是朝 ROAD 目标（西南）飞** —— 这是 ROAD 阶段的"区域中心 + 道路先验"合成目标出错，或道路先验误导；
3. **0 号整轮 99.4% 在 ROAD** —— 你的 `patrol_hover_deadline_=5s` 是 PATROL 阶段机制，在 ROAD 不生效。

## 为什么 hover-skip 没救

代码层面：

```cpp
// area_mission_node.cpp:2400 hover 块
if (mode_ == PATROL && !return_to_pause_ && !paused_ && !wps->empty())
{
  // hover 死线检测、跳线
}
```

`patrol_advance_*`（line 1855+）和 `patrol_stall_*`（line 2322+）也都在 PATROL 块内。三个跳过机制**只覆盖 PATROL 阶段**。

ROAD 阶段对应的保护只有：
- `enforceRegionBounds`（边界硬保护，margin=1.5m）
- `road_min_speed_floor`（速度下限）

但 `road_min_speed_floor` 是被安全层之后的软降速补丁，**前向速度下限只在"软降速 + 净空充足"时才兜底**。`enforceRegionBounds` 是硬停路径（在到达速度下限之前 `break` / 把 `vx=0`）—— 0 号属于"被边界硬停"，speed_floor 不生效。

## 建议的修复方向

按代价从小到大：

1. **0 号 ROAD 目标挪到区域中心安全点**（最快，最稳）：
   `(-35, -35)` → `(-25, -25)` 或 `(-30, -20)`，让 ROAD 阶段把无人机引导到区域内部而不是边界附近。

2. **enforceRegionBounds 的"只清单轴"改成"按距离缩比"**：
   当 `dist_to_edge < margin` 时，让 `|cmd.linear|` 整体按 `dist/margin` 线性 cap，而不是只清 vx。0 号同时朝正东 + 朝南跑时，整体速度被压到 `0.5 × 1.5 = 0.75 m/s`，而不是只剩 vy 的 1.5 m/s 在动。

3. **ROAD 阶段也加 hover-skip**：
   把 `patrol_hover_deadline_` 的判定从 `mode_ == PATROL` 改成 `(mode_ == PATROL || mode_ == ROAD) && !region_return_active_`，让 ROAD 阶段超过 N 秒 v_pub≈0 也强制跳进下一阶段或换目标。

4. **enforceRegionBounds margin 参数化**：
   把 `const double margin = 1.5;` 改成 `pnh_.param<double>("region_hard_margin", region_hard_margin_, 1.5);`，并与 `region_margin_` 联动（如果 `region_margin_` 是 6m，硬保护就 6m；至少参数可控）。

5. **enforceRegionBounds 增加单轴 speed cap 而非清零**：
   `vx = std::min(vx, 0.5 * (wx - (region_xmax_ - margin)))`，让接近边界时世界系 vx 平滑衰减到 0，而不是硬清零；同样能消除"贴边斜滑"。

## 待决策

修复方向 1 最稳但治标（0 号以后还会卡别的位置）；方向 2 治本但要改 MPPI 接口；方向 3 简单但要在 ROAD 阶段小心（避免误跳过正常恢复）。请告诉我选哪几个，我排个 plan 一次改完。
