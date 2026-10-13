# gzserver 崩溃取证手册（2026-09-28 整理）

> 本文档只讲"怎么抓现场、抓完看什么"，**不改任何启动脚本**。
> 所有命令都在 `start_referee.sh` 所在的同一台机器、同一用户（lishu2）下执行。

## 0. 本机现状（先读这段，能省一天）

| 项目 | 实测值 | 命令 |
|---|---|---|
| `core_pattern` | `\|/usr/share/apport/apport -p%p -s%s -c%c -d%d -P%P -u%u -g%g -F%F -- %E` | `cat /proc/sys/kernel/core_pattern` |
| apport | `enabled=1` | `grep ^enabled /etc/default/apport` |
| `ulimit -c` | agent shell 里是 `0`；**请在你自己的终端里复核**（harness 可能强制为 0） | `ulimit -c` |
| 遗留 core | **有**：`/var/crash/_usr_bin_gzserver-11.15.1.1000.crash`（9/22 22:13，81 MB，权限 `000`）、`_usr_bin_gzclient-11.15.1.1000.crash`（9/24，307 MB） | `ls -la /var/crash` |
| dmesg | 可读（普通用户即可） | `dmesg -T \| tail -30` |
| `/` 剩余空间 | 18 GB（单个 core 可达数百 MB，先看空间） | `df -h /` |

**关键陷阱：本机 `core_pattern` 是管道到 apport，不是文件名。** 所以"`ulimit -c unlimited` 之后 core 落在当前目录"在这台机器上**不成立**——原始 core 会被 apport 接管，产出的是 `/var/crash/*.crash`（apport 报告格式，内含 CoreDump 字段）。

dmesg 已有的历史证据（2026-09-28 一天内 **9 次** gzserver 崩溃）：

```
[一 9月 28 15:38:58 2026] gzserver[6150]: segfault at 7f1c0020bfd0 ip 00007f1c0020bfd0 sp 00007fff34b1bab8 error 15
[一 9月 28 16:03:46 2026] gzserver[76666]: segfault at 7fc144381ff0 ip 00007fc144381ff0 sp 00007fff081c7a18 error 15
[一 9月 28 17:13:11 2026] gzserver[418188]: segfault at 7ff58c3522d0 ip 00007ff58c3522d0 sp 00007ffd7fa33848 error 15
[一 9月 28 17:36:17 2026] traps: gzserver[687106] general protection fault ip:7fbcb7e10689 ... in libgazebo_physics.so.11.15.1
[一 9月 28 17:48:43 2026] gzserver[705312]: segfault at 7f72ac355d90 ip 00007f72ac355d90 sp 00007ffc542d6e38 error 15
[一 9月 28 18:03:21 2026] gzserver[851398]: segfault at 7f540037dc30 ip 00007f540037dc30 sp 00007fff8d4ac998 error 15
[一 9月 28 18:26:21 2026] gzserver[1013753]: segfault at 7fbf38f95d10 ip 00007fbf38f95d10 sp 00007ffd94de4898 error 15
[一 9月 28 18:53:30 2026] gzserver[1122534]: segfault at 7fd2b037a8d0 ip 00007fd2b037a8d0 sp 00007ffe4cab6128 error 15
[一 9月 28 19:19:39 2026] gzserver[1188299]: segfault at 7fc088379bf0 ip 00007fc088379bf0 sp 00007ffe93491f08 error 15
```

- 19:19:39 那次 = `logs/referee_only_20260928_191538/gzserver.log` 末尾的 `Segmentation fault (core dumped)`（该轮 gzserver 日志最后一条 sim time 211.328 s）。
- 8 次签名完全一致：**`ip == 故障地址`**，即 CPU 试图执行"数据地址"上的字节（控制流被破坏：vtable / 函数指针 / 回调被写坏 → use-after-free 或堆越界）。19:19 那次的 `Code:` 字节里 RIP 处是一段**运行期字符串**（`...link on the parent link.` 后面紧跟 `0x441` 和两个堆指针），该字符串不在任何 `.so` 的静态常量里 → 印证"跳进了堆里的对象"。
- `error 15` = `P|W|U|RSVD`，RSVD 位被置上通常意味着页表/指针已被彻底写坏。

## 1. 抓现场的三条路线（按推荐顺序）

### 路线 A：gdb 附着（最省事、能拿到活现场）
在 sim 跑起来之后，另一个终端：

```bash
PID=$(pgrep -f 'gzserver.*world_clean' | head -1)     # 或 pgrep -x gzserver
gdb -p $PID -ex 'set pagination off' -ex 'thread apply all bt' -ex 'detach' -ex quit   # 先来一份"健康时"的线程快照
# 想抓崩溃瞬间（gdb 会一直挂在崩溃点）：
gdb -p $PID -ex 'set pagination off' -ex continue      # 崩溃后 gdb 接管，再手动敲下面这些
```
崩溃后依次执行：
```
bt                      # 当前(崩溃)线程
thread apply all bt     # 所有线程（关键：看谁在写谁在读）
info registers
x/8i $pc-16             # 确认 RIP 前后是不是"数据"
x/s $pc                 # 如果是字符串，直接看出是谁的数据
```
注意：attach 会让 gzserver 停几百 ms~数秒，只是掉实时性、不会崩；dmesg 会留一条 `ptrace attach ...` 记录（9/28 18:25 已有两次）。

### 路线 B：吃 apport（不用 root，全自动）
```bash
ulimit -c unlimited        # 必须在"启动 start_referee.sh 的那个 shell"里执行，子进程才继承
./start_referee.sh         # 照常跑
# 崩溃后：
ls -lt /var/crash/ | head
tail -50 /var/log/apport.log      # apport 自己丢 core 的原因也写在这
```
取出可被 gdb 读的原始 core：
```bash
chmod u+r /var/crash/_usr_bin_gzserver-11.15.1.1000.crash     # 遗留文件是 000，owner 是你本人，不需要 sudo
apport-unpack /var/crash/_usr_bin_gzserver-11.15.1.1000.crash /tmp/gzcrash
ls /tmp/gzcrash            # 里面有 CoreDump / ExecutablePath / ProcCmdline / Signal / Date ...
gdb /usr/bin/gzserver /tmp/gzcrash/CoreDump -ex 'set pagination off' -ex 'bt' -ex 'thread apply all bt full'
```
> 先用 9/22 那份遗留 core 练手：它是 **9/17 修好 gps 插件之后**产生的，也就是说 GPS 数据竞态不是唯一根因，这份 core 值得直接读。

### 路线 C：让 raw core 落到工作区（需要 root 改内核参数，最干净）
```bash
mkdir -p /home/lishu2/2025_ZZU_FLY/logs/cores
sudo sysctl -w kernel.core_pattern=/home/lishu2/2025_ZZU_FLY/logs/cores/core.%e.%p.%t
cat /proc/sys/kernel/core_pattern      # 复核
```
之后 core 会直接落到 `logs/cores/`，`gdb /usr/bin/gzserver logs/cores/core.gzserver.<pid>.<ts>` 即可。
（`kernel.core_pattern` 是全局的，别忘记实验结束后改回 apport 那行。）

## 2. 留证清单（崩溃后 60 秒内做完，别急着重启）
```bash
ROUND=logs/<本轮目录>
dmesg -T | tail -60                        > $ROUND/dmesg_crash.txt
cp -a /var/crash/_usr_bin_gzserver-*.crash  $ROUND/ 2>/dev/null
cat /proc/sys/kernel/core_pattern          >> $ROUND/dmesg_crash.txt
tail -5 $ROUND/gzserver.log                >> $ROUND/dmesg_crash.txt   # 最后一条 sim time 很重要
grep -c "segfault" $ROUND/dmesg_crash.txt
```
`gzserver.log` 的最后一条 `[INFO] [... , <sim_time>]` 给出"崩溃发生时的仿真时刻"，用来和其它日志（referee/tracker）对齐。

## 3. 拿到 bt 之后重点看哪些帧
- **libprotobuf / `google::protobuf::MessageLite` / `Arena` / `internal::FunctionClosure`**：消息生命周期被破坏（对象在回调/发布途中被释放）。本项目 `/gazebo/model_states` 每秒要序列化 ~91 个模型 × 250 次，是最可疑的分配热点。
- **`libgazebo_ros_api_plugin.so` 的 `publishModelStates` / `publishLinkStates` / `deleteModel` / `getModelState`**：与"`delete_model` 之后模型 ~20 s 才消失 / 裁判主循环卡在 409.492"那条线索合流。注意 2026-09-28 起已把两个 topic 限频到 15 Hz（见下），若限频后崩溃消失即可反证。
- **`libgazebo_gps_plugin.so` 的 `GpsPlugin::OnSensorUpdate/OnWorldUpdate`**：9/17 已加锁修好（`tools/patches/apply_gps_plugin_fix.py --check` → "已经是修好的状态"）；若还在这里崩，说明还有第二处竞态。
- **`libgazebo_physics.so`**：17:36 的 GPF 就在这个库里（`bulletLink` / `parent link must be valid` 是它的断言字符串），指向物理引擎里 link/model 被并发删除。
- **actor 相关**（`libros_actor_cmd_pose_plugin.so`、`libActorCollisionsPlugin.so`）：actor 增删路径。
- 判读原则：**主线程在 protobuf/发布路径、另一线程在 model/link 删除路径** → 并发访问已释放对象。

## 4. 复现率与最小化（不改启动脚本）
- 崩溃与"订阅 `/gazebo/model_states` 的节点数量 × CPU 压力"正相关（裁判轮 19:15 那一轮 4 min 内必崩）。
- 最小复现可在工作区单独起：`env -u DISPLAY gzserver --verbose <world>`（headless 必须 `env -u DISPLAY`），再 `rosbag record /gazebo/model_states` 制造订阅者。
- 已做过的降载改动（2026-09-28）：`gazebo_ros_api_plugin` 新增 `~pub_model_states_rate` / `~pub_link_states_rate`（默认 15 Hz，≤0 = 每个世界更新周期都发），启动日志会打印实际生效频率：
  `XTDrone rate limit: /gazebo/model_states at 15.000 Hz, /gazebo/link_states at 15.000 Hz`。

## 5. 坑位备忘
- `ulimit -c unlimited` + apport 管道 = 不会有 `./core`；要 raw core 必须走路线 C。
- 别 `gdb --args roslaunch ...`：roslaunch 是 python，真正要 attach 的是它的子进程 `gzserver`。
- core 很大，`ulimit -c unlimited` 前先 `df -h /`；也可用 `ulimit -c 2000000`（≈2 GB 上限）折中。
- gzserver 崩了之后 `/gazebo/*` 服务全没了，裁判脚本会走 `abort_round`；**先取证再重启**。
- `/var/crash` 里 9/22、9/24 的遗留文件权限是 `000`，读之前 `chmod u+r`。
