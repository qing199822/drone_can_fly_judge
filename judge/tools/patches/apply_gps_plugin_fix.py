#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复 PX4 sitl_gazebo 的 gazebo_gps_plugin 数据竞态/空队列 UB（SIGABRT 根因）。

背景（2026-09-17 gdb 实证）：
    Thread 114 "gzserver" received signal SIGABRT
    double free or corruption (out)
    #4 _int_free -> #7 _Deque_base<SITLGps>::_M_deallocate_node
    #9 std::queue<SITLGps>::pop -> #11 gazebo::GpsPlugin::OnSensorUpdate()
    Tools/sitl_gazebo/src/gazebo_gps_plugin.cpp:342

根因（两处，都在同一队列上）：
    1) push 在 GpsPlugin::OnWorldUpdate()（世界/物理线程），front()/pop() 在
       GpsPlugin::OnSensorUpdate()（传感器线程），**没有任何锁** → deque 内部节点被并发改坏；
    2) OnSensorUpdate() 里先 front() 后判 empty() → 空队列时本身就是 UB；
       而且队列为空时仍会把默认构造的 SITLGps（time_usec=0, lat=lon=0）发布出去，
       会让 EKF 吃到一次坏 GPS（位姿跳变/异常）。

本脚本只改这两个文件，其它一律不动（可用 --revert 还原）：
    include/gazebo_gps_plugin.h
    src/gazebo_gps_plugin.cpp

用法：
    python3 tools/patches/apply_gps_plugin_fix.py                 # 打补丁（自动备份 .orig_20260918）
    python3 tools/patches/apply_gps_plugin_fix.py --revert        # 还原
    python3 tools/patches/apply_gps_plugin_fix.py --check         # 只检查是否已修/可修
环境变量 PX4_DIR 可指定 PX4 根目录（默认 $HOME/PX4-Autopilot，$HOME/PX4_Firmware 亦可）。
"""
import argparse
import os
import shutil
import sys

H_MUTEX_INCLUDE = ('#include <queue>', '#include <mutex>\n#include <queue>')
H_MEMBER_OLD = '  std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer;'
H_MEMBER_NEW = '''  std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer;
  // 2026-09-18: push 在世界线程(OnWorldUpdate)，pop 在传感器线程(OnSensorUpdate)，
  // 原实现无锁 + 先 front() 后判 empty() -> 堆损坏/SIGABRT（gdb 已抓到）。
  mutable std::mutex gps_buffer_mutex_;'''
C_PUSH_OLD = '''  // add msg to buffer
  gps_delay_buffer.push(gps_msg);'''
C_PUSH_NEW = '''  // add msg to buffer
  {
    std::lock_guard<std::mutex> lock(gps_buffer_mutex_);
    gps_delay_buffer.push(gps_msg);
  }'''
C_LOOP_OLD = '''    while (true) {
      gps_msg = gps_delay_buffer.front();
      double gps_current_delay = current_time_.Double() - gps_delay_buffer.front().time_usec() / 1e6f;

      if (gps_delay_buffer.empty()) {
        // abort if buffer is empty already
        break;
      }
      // remove data that is too old or if buffer size is too large
      if (gps_current_delay >= gps_delay) {
        gps_delay_buffer.pop();
      // remove data if buffer too large
      } else if (gps_delay_buffer.size() > gps_buffer_size_max) {
        gps_delay_buffer.pop();
      } else {
        // if we get here, we have good data, stop
        break;
      }
    }
    // publish SITLGps msg at the defined update rate
    gps_pub_->Publish(gps_msg);'''
C_LOOP_NEW = '''    bool has_msg = false;
    {
      std::lock_guard<std::mutex> lock(gps_buffer_mutex_);
      // 先判空再操作（原实现先 front() 后 empty()，空队列时是 UB）
      while (!gps_delay_buffer.empty()) {
        const double gps_current_delay =
            current_time_.Double() - gps_delay_buffer.front().time_usec() / 1e6f;
        if (gps_current_delay >= gps_delay ||
            gps_delay_buffer.size() > gps_buffer_size_max) {
          gps_msg = gps_delay_buffer.front();
          has_msg = true;
          gps_delay_buffer.pop();
        } else {
          gps_msg = gps_delay_buffer.front();   // 边界元素：延迟最接近 gps_delay
          has_msg = true;
          break;
        }
      }
    }
    // publish SITLGps msg at the defined update rate
    // 注意：队列为空时原实现会把默认构造的 gps_msg（time_usec=0 / lat=lon=0）也发出去，
    // 会让 EKF 收到一次"零时刻零坐标"的坏 GPS（位姿跳变）。这里不再发。
    if (has_msg && gps_pub_) {
      gps_pub_->Publish(gps_msg);
    }'''
BACKUP_SUFFIX = '.orig_20260918'


def find_px4(explicit=None):
    candidates = [explicit] if explicit else []
    for env in ('PX4_DIR',):
        if os.environ.get(env):
            candidates.append(os.environ[env])
    home = os.path.expanduser('~')
    candidates += [os.path.join(home, 'PX4-Autopilot'), os.path.join(home, 'PX4_Firmware')]
    for path in candidates:
        if path and os.path.isdir(os.path.join(path, 'Tools', 'sitl_gazebo')):
            return os.path.realpath(path)
    sys.exit('找不到 PX4 目录（试过：%s）；可用 PX4_DIR=/path/to/PX4-Autopilot 指定' % candidates)


def patch_text(text, pairs):
    """按顺序做精确替换；若某处已替换过则跳过。返回 (新文本, 是否需要改)。"""
    changed = False
    for old, new in pairs:
        if new in text:          # 已经是修好的形态
            continue
        if old not in text:
            return text, None    # 结构与预期不符
        text = text.replace(old, new, 1)
        changed = True
    return text, changed


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-dir', default=None)
    parser.add_argument('--revert', action='store_true')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args(argv)
    px4 = find_px4(args.px4_dir)
    gazebo = os.path.join(px4, 'Tools', 'sitl_gazebo')
    files = {'h': os.path.join(gazebo, 'include', 'gazebo_gps_plugin.h'),
             'c': os.path.join(gazebo, 'src', 'gazebo_gps_plugin.cpp')}
    print('[gps-fix] PX4 = %s' % px4)

    if args.revert:
        for key, path in files.items():
            backup = path + BACKUP_SUFFIX
            if not os.path.exists(backup):
                print('[gps-fix] 没有备份 %s，跳过还原' % backup)
                continue
            shutil.copy2(backup, path)
            print('[gps-fix] 已还原 %s' % os.path.relpath(path, px4))
        return 0

    texts = {k: open(p, encoding='utf-8').read() for k, p in files.items()}
    patched = {}
    for key, pairs in (('h', (H_MUTEX_INCLUDE, (H_MEMBER_OLD, H_MEMBER_NEW))),
                       ('c', ((C_PUSH_OLD, C_PUSH_NEW), (C_LOOP_OLD, C_LOOP_NEW)))):
        new_text, changed = patch_text(texts[key], pairs)
        if changed is None:
            print('[gps-fix] !! %s 的结构与预期不符（可能已被改过或是别的 PX4 版本），未修改任何文件'
                  % os.path.relpath(files[key], px4))
            return 2
        patched[key] = (new_text, changed)

    if not any(changed for _, changed in patched.values()):
        print('[gps-fix] 已经是修好的状态，无需修改')
        return 0
    if args.check:
        print('[gps-fix] 可应用（--check 未写入）')
        return 0

    for key, path in files.items():
        backup = path + BACKUP_SUFFIX
        if not os.path.exists(backup):
            shutil.copy2(path, backup)
            print('[gps-fix] 备份 %s' % os.path.relpath(backup, px4))
        new_text, changed = patched[key]
        if changed:
            open(path, 'w', encoding='utf-8').write(new_text)
            print('[gps-fix] 已修补 %s' % os.path.relpath(path, px4))
    print('[gps-fix] 完成。重新编译：')
    print('    cd %s/build/px4_sitl_default/build_gazebo && ninja libgazebo_gps_plugin.so' % px4)
    print('[gps-fix] 还原：python3 %s --revert' % os.path.abspath(__file__))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
