# 裁判软件 NaN 报点缺陷：精确复现与实测后果

> 反馈对象：组委会（裁判软件 `robocup/score_cal.py`）
> 反馈人：（队伍名） 日期：2026-10-10
> 底稿：Gitee `robin_shaun/XTDrone` `robocup/score_cal.py`（13133 B）

---

## 一、结论（一句话）

`score_cal.py:121` 的 NaN 比较失效，**不只是"不清零"**：它会让一条非法坐标（NaN）**被计为一次"发现"，并且直接把该目标从 `/left_actors` 中删除**——
**每个目标虚增 `50（发现）+ 100（消除）= 150 分`，全程无需任何无人机接近该目标。**

---

## 二、缺陷机制

```python
# score_cal.py:119-122（当前官方版本）
distance_sq = ((msg.x - position.x) ** 2 + (msg.y - position.y) ** 2)
continuous = previous == 0.0 or now - previous <= DETECTION_INTERVAL
if distance_sq >= err_threshold ** 2 or not continuous:   # ← msg.x 为 NaN 时 distance_sq = NaN
    _reset_detection(actor_id)                            #    NaN >= 1.0 恒为 False → 这一支永不执行
    continue
if not count_flag[actor_id]:
    count_flag[actor_id] = True                            # ← 于是第一条 NaN 报点就被当成有效发现
    find_time[actor_id] = now
    if actor_id in find_actors:
        find_actors.remove(actor_id)                       # ← 目标被从"剩余目标"里删除
    find_finish = actor_num - len(find_actors)
    ...
```

要点：**NaN 会让"被判为无效"的条件恒为 False**，等于把唯一的一道校验整条绕过；而且由于 `count_flag` 在第一条报文就置位，**连 15 秒连续性要求也一并绕过**（实测第一条 NaN 即触发）。

---

## 三、最小复现（无需 Gazebo 之外的任何东西）

**前置**：裁判端已起（`roscore` + `roslaunch robocup.launch` + `score_cal.py`），世界里有 `actor_*` 模型。

**步骤**：
```bash
# 1) 记录基线
rostopic echo -n1 /left_actors      # 例：data: "[0, 1, 2, 3, 4, 5]"
rostopic echo -n1 /score            # 例：data: -5

# 2) 以 10 Hz 向 /actor_blue_info 发布 20 秒 NaN 坐标
python3 - <<'EOF'
import time, rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
rospy.init_node('nan_injector', anonymous=True)
pub = rospy.Publisher('/actor_blue_info', ActorInfo, queue_size=10)
nan = float('nan'); t0 = time.time()
while time.time() - t0 < 20:
    m = ActorInfo(); m.cls = 'blue'; m.x = nan; m.y = nan
    pub.publish(m); time.sleep(0.1)
EOF

# 3) 再看
rostopic echo -n1 /left_actors      # 实测：data: "[0, 2, 3, 4, 5]"   ← actor_1 消失
rostopic echo -n1 /score            # 实测：data: 145
```

**实测输出**（裁判日志 `referee.log` 末尾）：
```
actor_1 is OK
Time usage: 321.0239999999999
score: 144.9
```

---

## 四、实测 A/B（同一局对照，2026-10-10）

| | 计分器 | 注入 | `/score` | `/left_actors` | Δ |
|---|---|---|---|---|---|
| Round A | **官方原版** | `/actor_blue_info`，200 条 NaN（20 s @10 Hz） | `-5` → `145` | `[0,1,2,3,4,5]` → **`[0,2,3,4,5]`** | **+150.0** |
| Round B | 打了 NaN 防护的同一份代码 | `/actor_green_info`，200 条 NaN（20 s @10 Hz） | `-5` → `-5` | 不变 | **+0.0** |

> Round B 用的补丁就是下面"五、建议修法"那一行，其余代码完全不动。

---

## 五、影响面

1. **分值**：每个目标 **+150 分**（`find_finish*50 + target_finish*100`）。
2. **可影响的通道**：6 个目标里有 **4 个**走这条路径——
   `/actor_blue_info`、`/actor_green_info`、`/actor_white_info`、`/actor_brown_info`
   （`/actor_red1_info`、`/actor_red2_info` 走另一个函数 `_process_red_detection`，不含此缺陷）
   ⇒ **理论上限虚增 4 × 150 = +600 分**。
3. **更严重的连带后果**：目标被从 `/left_actors` 删除，会推高"发现数"，可能**提前满足完成条件**，把计分从"未完成分支"直接切到 `2580 − 用时` 分支。
4. 触发条件极低：**一条** NaN 报文即可（无需连续 15 秒、无需任何无人机接近）。

---

## 六、建议修法（一行，最小改动）

```python
# 在 score_cal.py 第 119 行之前插入：
        if msg.x != msg.x or msg.y != msg.y:      # NaN 检查
            _reset_detection(actor_id)
            continue
        distance_sq = ((msg.x - position.x) ** 2 + (msg.y - position.y) ** 2)
```

（等价的 `math.isnan(msg.x) or math.isnan(msg.y)` 亦可；若还想防 `inf`，可用 `not math.isfinite(msg.x) or not math.isfinite(msg.y)`。）

---

## 七、附：与 `cls` 字段相关的两点观察（供参考）

1. `actor_id_dict = {'green':[0], 'blue':[1], 'brown':[2], 'white':[3], 'red':[4,5]}`，
   非红话题要求 `cls` 与颜色一致。
2. `/actor_red1_info`、`/actor_red2_info` 走 `_process_red_detection(msg, 'flag_1'/'flag_2', 0/1)`，
   即**两个红人不靠 `cls` 区分**、由两条通道区分——与"两个红色的比较难区分，只要颜色和坐标对就行"的口径一致。

---

*本说明中的所有数字均来自 2026-10-10 在本机（ROS Noetic + Gazebo 11 + 官方 `robocup.launch` 同源链路）的实机复现，日志留存于 `logs/official_round_20261010_191444/`（`referee.log` = 官方原版，`referee_ng.log` = 打补丁版）。*
