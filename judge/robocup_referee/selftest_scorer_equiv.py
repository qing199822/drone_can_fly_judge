#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评分状态机等价测试：同输入喂给上游 score_cal.py 与 referee.py，比结果是否一致。

做不了逐字节对比（score_cal 是 ROS 节点），所以改用"决策等价"：
  * 用 importlib 把 **XTDrone/robocup/score_cal.py 本体** 当模块加载（它所有 ROS 调用
    都在 `if __name__ == "__main__"` 里，import 不会启动节点）；
  * 把它的 rospy / 服务 / 发布者全部换成假的（时钟可控、del_model 恒成功、
    SCORE_OUTPUT_PATH 指到临时目录，避免写进仓库）；
  * 然后用**同一串输入**分别驱动它和 RefereeScorer，逐步比对：
      find_finish / target_finish / left_actors / score / count_flag /
      tracked_actors / tracking_start_time / 最终分落盘
  * 另外把计分公式在一个参数矩阵上逐点比对（含完成时 2580−用时 分支）。

覆盖不到的部分（都在上游 main 循环里、不是函数，只能靠读代码对齐）：
  高度 >6m 判 0、600s 超时、UAV 失联扣分、upstream 的 finish 语义。
用法：
    python3 robocup_referee/selftest_scorer_equiv.py
退出码 0 = 一致。
"""

import importlib.util
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(HERE)
UPSTREAM = os.path.join(WORKSPACE, 'XTDrone', 'robocup', 'score_cal.py')

sys.path.insert(0, HERE)
import referee as _referee   # noqa: E402

# 上游 score_cal.py 要 import 自己构建的消息包（在 devel/lib/python3/dist-packages）
_referee.ensure_ros_log_dir()
_referee.bootstrap_catkin_paths()


# --------------------------------------------------------------------------
# 假件
# --------------------------------------------------------------------------
class Clock(object):
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeRosPy(object):
    """只提供 upstream 用到的两个入口。"""

    def __init__(self, clock):
        self.clock = clock
        self.shutdown_reasons = []

    def get_time(self):
        return self.clock()

    def signal_shutdown(self, reason=''):
        self.shutdown_reasons.append(reason)


class FakePub(object):
    def __init__(self):
        self.values = []

    def publish(self, value):
        self.values.append(value)


class FakeDelResp(object):
    def __init__(self, ok=True):
        self.success = ok
        self.status_message = ''


class Msg(object):
    def __init__(self, cls, x, y):
        self.cls, self.x, self.y = cls, x, y


class Point(object):
    def __init__(self, x, y, z=1.25):
        self.x, self.y, self.z = x, y, z


def load_upstream():
    spec = importlib.util.spec_from_file_location('upstream_score_cal', UPSTREAM)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prepare_upstream(up, clock, tmp):
    """把 upstream 模块的全局状态摆成"刚启动"的样子。"""
    up.rospy = FakeRosPy(clock)
    up.SCORE_OUTPUT_PATH = os.path.join(tmp, 'upstream_score.txt')
    up.left_actors = list(range(6))
    up.find_actors = list(range(6))
    up.actors_pos = [None] * 6
    up.count_flag = [False] * 6
    up.topic_arrive_time = [0.0] * 6
    up.find_time = [0.0] * 6
    up.tracking_start_time = [None] * 6
    up.tracked_actors = set()
    up.uav_positions = [None] * 6
    up.uav_lost = [False] * 6
    up.uav_seen = [False] * 6
    up.uav_miss_count = [0] * 6
    up.mission_finished = False
    up.mission_completed = False
    up.uav_loss_count = 0
    up.uav_loss_penalty = 100.0
    up.target_finish = 0
    up.find_finish = 0
    up.score = 0.0
    up.time_usage = 0.0
    up.start_time = clock()
    up.sensor_cost = 1700.0
    up.find_actor_pub = [FakePub() for _ in range(6)]
    up.score_pub = FakePub()
    up.time_usage_pub = FakePub()
    up.left_actors_pub = FakePub()
    up.del_model = lambda name: FakeDelResp(True)


def prepare_mine(clock, tmp, pub=None):
    # dump_score=False：分数落盘是本程序附加的，上游没有，不参与等价比对
    cfg = _referee.Config(score_mode='repo', dump_score=False)
    sc = _referee.RefereeScorer(cfg, clock=clock,
                                publish_score=(pub.publish if pub else None))
    sc.start(clock())
    return sc


def snapshot(up, mine, with_tracking=False):
    keys = ['find_finish', 'target_finish', 'left_actors', 'count_flag']
    if with_tracking:
        keys.append('tracked_actors')
    bad = []
    for key in keys:
        a = getattr(up, key)
        b = getattr(mine, key)
        if isinstance(a, set) or isinstance(b, set):
            a, b = sorted(a), sorted(b)
        if a != b:
            bad.append('%s: upstream=%r mine=%r' % (key, a, b))
    if abs(up.score - mine.score) > 1e-6:
        bad.append('score: upstream=%.6f mine=%.6f' % (up.score, mine.score))
    if with_tracking:
        a, b = up.tracking_start_time, mine.tracking_start_time
        if [None if v is None else round(v, 6) for v in a] != \
           [None if v is None else round(v, 6) for v in b]:
            bad.append('tracking_start_time: upstream=%r mine=%r' % (a, b))
    return bad


def main():
    if not os.path.isfile(UPSTREAM):
        print('找不到上游脚本：%s' % UPSTREAM)
        return 2
    tmp = tempfile.mkdtemp(prefix='referee_scorer_equiv_')
    failures = []
    stats = {'n': 0}

    def report(name, bad):
        stats['n'] += 1
        if bad:
            failures.append(name)
            print('  [FAIL] %s' % name)
            for line in bad:
                print('         %s' % line)
        else:
            print('  [PASS] %s' % name)

    print('上游：%s\n' % UPSTREAM)
    try:
        # ---------------- 场景 1：发现 → 连续保持 15s → 消除 ----------------
        clock = Clock(0.0)
        up = load_upstream()
        prepare_upstream(up, clock, tmp)
        mine = prepare_mine(clock, tmp)
        up.actors_pos[0] = Point(10.0, 10.0)
        mine.actors_pos[0] = Point(10.0, 10.0)
        msg = Msg('green', 10.0, 10.0)
        bad = []
        t = 0.0
        while t <= 16.0:
            clock.t = t
            up._process_actor_detection(msg, [0])
            mine.process_actor_detection(msg, [0])
            bad += ['t=%.1f %s' % (t, b) for b in snapshot(up, mine)]
            t += 0.5
        report('场景1 发现→保持15s→消除（33 步逐步对比）', bad)

        # ---------------- 场景 2：跟踪 20 秒 ----------------
        clock = Clock(0.0)
        up = load_upstream()
        prepare_upstream(up, clock, tmp)
        mine = prepare_mine(clock, tmp)
        up.actors_pos[0] = Point(10.0, 10.0)
        mine.actors_pos[0] = Point(10.0, 10.0)
        up.uav_positions[0] = Point(12.0, 10.0)
        mine.uav_positions[0] = Point(12.0, 10.0)
        bad = []
        for t in (0.0, 5.0, 19.9, 20.0, 20.5, 40.0):
            clock.t = t
            up._update_tracking(t)
            mine.update_tracking(t)
            bad += ['t=%.1f %s' % (t, b) for b in snapshot(up, mine, with_tracking=True)]
        # 再让 UAV 飞远，验证"已得的跟踪分不会取消"
        up.uav_positions[0] = Point(999.0, 999.0)
        mine.uav_positions[0] = Point(999.0, 999.0)
        clock.t = 41.0
        up._update_tracking(41.0)
        mine.update_tracking(41.0)
        bad += ['远离后 %s' % b for b in snapshot(up, mine, with_tracking=True)]
        report('场景2 连续跟踪 20s→+80，且远离后不取消', bad)

        # -------------- 场景 1b：6 个人物全消除 → 2580 − 用时 --------------
        # 上游把 2580 公式 inline 在"全消除"那一处（不在 _progress_score 里），
        # 所以必须真的把 6 个都消掉才能比到它。
        clock = Clock(0.0)
        up = load_upstream()
        prepare_upstream(up, clock, tmp)
        mine = prepare_mine(clock, tmp)
        positions = {i: (10.0 + 5.0 * i, 10.0) for i in range(6)}
        for i, (x, y) in positions.items():
            up.actors_pos[i] = Point(x, y)
            mine.actors_pos[i] = Point(x, y)
        t = 0.0
        finish_t = None
        while t <= 16.5:
            clock.t = t
            for i, (x, y) in positions.items():
                m = Msg('green', x, y)
                up._process_actor_detection(m, [i])
                mine.process_actor_detection(m, [i])
            if finish_t is None and mine.mission_finished:
                finish_t = t
            t += 0.5
        bad = []
        if up.mission_finished != mine.mission_finished:
            bad.append('mission_finished: upstream=%s mine=%s'
                       % (up.mission_finished, mine.mission_finished))
        if abs(up.score - mine.score) > 1e-6:
            bad.append('全消除后 score: upstream=%.6f mine=%.6f'
                       % (up.score, mine.score))
        expected = 2580.0 - finish_t - 5.1      # 2580 − 用时 − 传感器×0.003
        if abs(mine.score - expected) > 1e-6:
            bad.append('本程序 score=%.6f 与 2580−用时(%.1f)−5.1=%.6f 不符'
                       % (mine.score, finish_t, expected))
        report('场景1b 6 个人物全消除 → inline 2580−用时（消除于 t=%.1f）' % finish_t,
               bad)

        # ---------------- 场景 3：计分公式矩阵 ----------------
        bad = []
        combos = []
        for find_finish in (0, 1, 6):
            for target_finish in (0, 3, 6):
                for n_tracked in (0, 2, 6):
                    for uav_loss in (0, 1, 3):
                        for completed in (False, True):
                            for elapsed in (0.0, 123.4, 600.0):
                                combos.append((find_finish, target_finish, n_tracked,
                                               uav_loss, completed, elapsed))
        for (ff, tf, nt, loss, completed, elapsed) in combos:
            clock = Clock(0.0)
            up = load_upstream()
            prepare_upstream(up, clock, tmp)
            mine = prepare_mine(clock, tmp)
            up.find_finish, up.target_finish = ff, tf
            up.tracked_actors = set(range(nt))
            up.uav_loss_count = loss
            up.mission_completed = completed
            up.start_time = 0.0
            clock.t = elapsed
            mine.find_finish, mine.target_finish = ff, tf
            mine.tracked_actors = set(range(nt))
            mine.uav_loss_count = loss
            mine.mission_completed = completed
            mine.start_time = 0.0
            a = up._progress_score()
            b = mine.progress_score()
            if abs(a - b) > 1e-6:
                bad.append('ff=%d tf=%d tracked=%d loss=%d done=%s elapsed=%g: '
                           'upstream=%.6f mine=%.6f'
                           % (ff, tf, nt, loss, completed, elapsed, a, b))
        report('场景3 计分公式矩阵（%d 组）' % len(combos), bad)

        # ---------------- 场景 4：发布到 /score 的整数值 ----------------
        clock = Clock(0.0)
        up = load_upstream()
        prepare_upstream(up, clock, tmp)
        up_pub = FakePub()
        up.score_pub = up_pub
        mine_pub = FakePub()
        mine = prepare_mine(clock, tmp, pub=mine_pub)
        bad = []
        for value in (1234.7, -5.1, 0.0, 32767.6, -40000.0):
            up.score = mine.score = value
            up._publish_score()
            mine.publish_score()
            if up_pub.values[-1] != mine_pub.values[-1]:
                bad.append('score=%.1f: upstream=%s mine=%s'
                           % (value, up_pub.values[-1], mine_pub.values[-1]))
        report('场景4 发布到 /score 的整数值一致（含越界裁剪）', bad)
        print('  [INFO] 上游 score_cal.py 有 _dump_score_file 吗？%s'
              '（分数落盘是本程序附加的，可用 --no-score-dump 关掉）'
              % hasattr(up, '_dump_score_file'))

        print()
        if failures:
            print('评分等价测试：%d 项中 %d 项失败：%s'
                  % (stats['n'], len(failures), failures))
            return 1
        print('评分等价测试：%d 项全部一致（与上游 score_cal.py 本体逐步比对）' % stats['n'])
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
