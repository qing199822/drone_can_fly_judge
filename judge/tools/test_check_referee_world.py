#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/check_referee_world.py 的回归测试（纯逻辑，不需要 ROS / Gazebo）。

重点锁住评审提的 P1：
  1. 固定图**没有** black_box.txt 时，必须从 world 推导建筑框并真的做占位检查
     —— 不能把"没有 black_box.txt"当成检查通过；
  2. 推导不出来任何框时必须 FAIL（检查不完整，不许放行）；
  3. 人物落在推导出的建筑框里必须 FAIL；
  4. 提供 black_box.txt 时以它为准（随机图的路径不变）；
  5. 真实 base0.world（先 dedupe 掉 <state>）必须能通过。

运行：python3 tools/test_check_referee_world.py
"""

import importlib.util
import os
import subprocess
import sys
import tempfile
import textwrap
import unittest

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TOOLS)
CHECKER = os.path.join(TOOLS, 'check_referee_world.py')
DEDUPER = os.path.join(TOOLS, 'dedupe_world.py')


def load_checker():
    spec = importlib.util.spec_from_file_location('check_referee_world', CHECKER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CHK = load_checker()

# 一个最小 world：一栋 house_1_66（16x12）+ 6 个人物 + 6 个 drone 出生点附近无遮挡
WORLD_TMPL = """<?xml version="1.0"?>
<sdf version="1.6">
<world name="default">
  <include><name>ground_plane</name><uri>model://ground_plane</uri></include>
  <model name="house_1_66">
    <static>1</static>
    <pose frame=''>{hx} {hy} 0 0 0 0</pose>
    <link name='link'><collision name='collision'><geometry><mesh><uri>model://house_1/meshes/house_1.dae</uri></mesh></geometry></collision></link>
  </model>
  <model name="lamp_post_1">
    <static>1</static>
    <pose frame=''>{lx} {ly} 0 0 0 0</pose>
    <link name='link'><collision name='collision'><geometry><mesh><uri>model://lamp_post/meshes/lamp_post.dae</uri></mesh></geometry></collision></link>
  </model>
{actors}
</world>
</sdf>
"""

ACTOR = """  <actor name="actor_{i}">
    <pose frame=''>0 0 1.25 0 0 0</pose>
    <plugin name="actor{i}_plugin" filename="libros_actor_cmd_pose_plugin.so">
      <init_pose>{x} {y} 1.25 1.57 0 0</init_pose>
    </plugin>
  </actor>
"""


def build_world(tmpdir, house=(60.0, 20.0), lamp=(-30.0, -30.0), actor_xy=None):
    actor_xy = actor_xy or [(-45, -45), (-45, 35), (30, 30), (30, -30), (80, -20), (110, 20)]
    actors = ''.join(ACTOR.format(i=i, x=xy[0], y=xy[1]) for i, xy in enumerate(actor_xy))
    path = os.path.join(tmpdir, 'test.world')
    with open(path, 'w') as handle:
        handle.write(textwrap.dedent(WORLD_TMPL).format(hx=house[0], hy=house[1],
                                                        lx=lamp[0], ly=lamp[1],
                                                        actors=actors))
    return path


class DeriveBoxesTest(unittest.TestCase):
    def test_derives_boxes_without_black_box(self):
        boxes, resolved, skipped = CHK._derive_boxes_from_world(
            __import__('xml.etree.ElementTree', fromlist=['ET']).parse(
                build_world(tempfile.mkdtemp())).getroot().find('world'))
        self.assertGreaterEqual(len(boxes), 2)          # house + lamp
        self.assertTrue(any('house_1_66' in n for n in resolved))
        # house_1_66 是 16x12 → AABB 半长 8/6
        house_box = [b for b in boxes if abs(b[0][0] - 52) < 0.01]
        self.assertTrue(house_box, '没有推算出 house_1_66 的框：%s' % boxes)
        self.assertAlmostEqual(house_box[0][0][0], 52.0)   # 60-8
        self.assertAlmostEqual(house_box[0][0][1], 68.0)   # 60+8

    def test_exact_box_geometry_is_used(self):
        world = """<?xml version="1.0"?><sdf version="1.6"><world name="w">
        <model name="weird_obstacle"><static>1</static><pose frame=''>5 6 0 0 0 0</pose>
        <link name='l'><collision name='c'><geometry><box><size>4 2 1</size></box></geometry></collision></link>
        </model></world></sdf>"""
        element = __import__('xml.etree.ElementTree', fromlist=['ET']).fromstring(world).find('world')
        boxes, resolved, _ = CHK._derive_boxes_from_world(element)
        self.assertEqual(len(boxes), 1)
        self.assertEqual(boxes[0], [[3.0, 7.0], [5.0, 7.0]])


class MainBehaviourTest(unittest.TestCase):
    def _run(self, *args):
        return subprocess.run([sys.executable, CHECKER] + list(args),
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    def test_actor_inside_derived_house_fails(self):
        tmp = tempfile.mkdtemp()
        # 人物放在 house_1_66 里面（60,20），应被推导框抓住
        world = build_world(tmp, actor_xy=[(60, 20)] + [(-45, -45), (-45, 35), (0, 40), (80, -20), (110, 20)])
        proc = self._run(world)
        out = proc.stdout.decode()
        self.assertEqual(proc.returncode, 1, out)
        self.assertIn('actor_0', out)
        self.assertIn('落在建筑', out)
        self.assertIn('world 推导', out)

    def test_clean_world_passes_without_black_box(self):
        tmp = tempfile.mkdtemp()
        world = build_world(tmp)
        proc = self._run(world)
        out = proc.stdout.decode()
        self.assertEqual(proc.returncode, 0, out)
        self.assertIn('结果：通过', out)
        self.assertIn('world 推导', out)

    def test_no_boxes_at_all_is_a_failure(self):
        """world 里没有任何可推导的障碍 → 检查不完整，必须 FAIL（不能当通过）。"""
        tmp = tempfile.mkdtemp()
        world = os.path.join(tmp, 'empty.world')
        with open(world, 'w') as handle:
            handle.write('<?xml version="1.0"?><sdf version="1.6"><world name="w">'
                         '<include><name>ground_plane</name>'
                         '<uri>model://ground_plane</uri></include></world></sdf>')
        proc = self._run(world)
        out = proc.stdout.decode()
        self.assertEqual(proc.returncode, 1, out)
        self.assertIn('检查不完整', out)

    def test_black_box_txt_takes_precedence(self):
        tmp = tempfile.mkdtemp()
        world = build_world(tmp)                       # house 在 (60,20)，人物都不在里面
        bb = os.path.join(tmp, 'black_box.txt')
        with open(bb, 'w') as handle:
            # 故意把 actor_2 (30,30) 圈进 black_box → 以 black_box 为准必须 FAIL
            handle.write(str([[(-50.0, 130.0), (20.0, 40.0)]]))
        proc = self._run(world, bb)
        out = proc.stdout.decode()
        self.assertEqual(proc.returncode, 1, out)
        self.assertIn('black_box.txt', out)
        self.assertIn('actor_2', out)


class RealBase0Test(unittest.TestCase):
    def test_base0_after_dedupe_passes(self):
        tmp = tempfile.mkdtemp()
        clean = os.path.join(tmp, 'base0_clean.world')
        raw = os.path.join(REPO, 'waypoint', 'world', 'base0.world')
        if not os.path.exists(raw):
            self.skipTest('没有 base0.world')
        subprocess.run([sys.executable, DEDUPER, raw, clean], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=True)
        proc = subprocess.run([sys.executable, CHECKER, clean],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = proc.stdout.decode()
        self.assertEqual(proc.returncode, 0, out)
        self.assertIn('world 推导', out)


if __name__ == '__main__':
    unittest.main(verbosity=2)
