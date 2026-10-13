#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复 PX4 sitl_gazebo 的 gazebo_groundtruth_plugin 在 gzserver 关机时崩溃（SIGSEGV）。

背景（2026-09-25 三份 apport core，栈逐帧一致）：
    #0 跳进 RW 无 X 的匿名堆 (si_addr==rip==rax，悬垂 vtable)
    #1 gazebo::physics::Model::Reset()            libgazebo_physics.so.11
    #3 gazebo::physics::World::Reset()
    #4 gazebo::GroundtruthPlugin::~GroundtruthPlugin()      <- 触发者
    #6 Model::Fini() -> #7 World::Fini() -> #10 gazebo::shutdown() -> Server::Run() -> main

根因：
    GroundtruthPlugin::~GroundtruthPlugin() 里调用了 world_->Reset()。
    关机时 gazebo::shutdown() -> World::Fini() 正在销毁 model，本插件析构又重入
    World::Reset()，遍历到已经析构的 link，经悬垂 vtable 跳到已释放的堆内存 -> SIGSEGV。
    现象：每次 Ctrl-C / 停止仿真都在退出时崩溃（Segmentation fault (core dumped)，
    dmesg 里 segfault at X ip X error 15），并会让仍在运行的裁判读到垃圾位姿从而清零分数。

修复（2026-09-28）：
    直接删除析构里的 world_->Reset() 调用。该调用的本意只是"插件卸载时顺手复位世界"，
    对本插件没有任何语义价值（插件不缓存世界状态，重新加载时 Load() 会重新取
    world_/model_），而它在关机路径下必然重入已析构对象。因为无法在 ModelPlugin 析构
    体内可靠判断"是否正在 shutdown"，加条件保护也只是把窗口缩小而不是消除，故最小且
    最安全的做法是删除该调用。

本脚本只改这一个文件，其它一律不动（可用 --revert 还原）：
    src/gazebo_groundtruth_plugin.cpp

用法：
    python3 tools/patches/apply_groundtruth_plugin_fix.py              # 打补丁（自动备份 .orig_20260928）
    python3 tools/patches/apply_groundtruth_plugin_fix.py --revert     # 逐字节还原
    python3 tools/patches/apply_groundtruth_plugin_fix.py --check      # 只检查，不写文件
环境变量 PX4_DIR 可指定 PX4 根目录（默认 $HOME/PX4-Autopilot，$HOME/PX4_Firmware 亦可）。
"""
import argparse
import os
import shutil
import sys

CPP_OLD = '''GroundtruthPlugin::~GroundtruthPlugin()
{
  if (updateConnection_)
    updateConnection_->~Connection();
  world_->Reset();
}'''
CPP_NEW = '''GroundtruthPlugin::~GroundtruthPlugin()
{
  if (updateConnection_)
    updateConnection_->~Connection();
  // 2026-09-28 修复 gzserver 关机崩溃：这里原本还有一句 world_->Reset();
  // 关机时 gazebo::shutdown() -> World::Fini() 正在销毁 model，本插件析构又重入
  // World::Reset()，遍历到已析构的 link，经悬垂 vtable 跳进已释放堆内存 -> SIGSEGV，
  // 于是每次 Ctrl-C / 停止仿真都在退出时崩溃（core dumped），还会让仍在运行的裁判
  // 读到垃圾位姿判 0 分。三份 9/25 完整 core 栈逐帧一致：
  //   #0 跳进 RW 无 X 的匿名堆 (si_addr==rip==rax，悬垂 vtable)
  //   #1 gazebo::physics::Model::Reset()            libgazebo_physics.so.11
  //   #3 gazebo::physics::World::Reset()
  //   #4 gazebo::GroundtruthPlugin::~GroundtruthPlugin()      <- 触发者
  //   #6 Model::Fini() -> #7 World::Fini() -> #10 gazebo::shutdown() -> Server::Run() -> main
  // 该调用的本意只是"插件卸载时顺手复位世界"，对地面真值插件没有语义价值（不缓存世界
  // 状态，重新加载时 Load() 会重新取 world_/model_），而它在关机路径下必然重入已析构
  // 对象；ModelPlugin 析构体内也无法可靠判断是否正在 shutdown，所以直接删除最安全。
}'''
BACKUP_SUFFIX = '.orig_20260928'


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


def patch_text(text):
    """做精确替换；返回 (新文本, 是否需要改)。changed=None 表示结构与预期不符。"""
    if CPP_NEW in text:          # 已经是修好的形态
        return text, False
    if CPP_OLD not in text:
        return text, None        # 结构与预期不符
    return text.replace(CPP_OLD, CPP_NEW, 1), True


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-dir', default=None)
    parser.add_argument('--revert', action='store_true')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args(argv)
    px4 = find_px4(args.px4_dir)
    path = os.path.join(px4, 'Tools', 'sitl_gazebo', 'src', 'gazebo_groundtruth_plugin.cpp')
    backup = path + BACKUP_SUFFIX
    print('[groundtruth-fix] PX4 = %s' % px4)

    if args.revert:
        if not os.path.exists(backup):
            print('[groundtruth-fix] 没有备份 %s，跳过还原' % backup)
            return 0
        shutil.copy2(backup, path)
        print('[groundtruth-fix] 已从 %s 逐字节还原 %s'
              % (os.path.basename(backup), os.path.relpath(path, px4)))
        return 0

    if not os.path.exists(path):
        print('[groundtruth-fix] !! 找不到 %s' % path)
        return 2
    text = open(path, encoding='utf-8').read()
    new_text, changed = patch_text(text)
    if changed is None:
        print('[groundtruth-fix] !! %s 的结构与预期不符（可能已被改过或是别的 PX4 版本），未修改任何文件'
              % os.path.relpath(path, px4))
        return 2

    if not changed:
        print('[groundtruth-fix] 已经是修好的状态（析构里没有 world_->Reset()），无需修改')
        return 0
    if args.check:
        print('[groundtruth-fix] 未修复但可应用（--check 未写入）：将删除 ~GroundtruthPlugin() 里的 world_->Reset()')
        return 0

    if not os.path.exists(backup):
        shutil.copy2(path, backup)
        print('[groundtruth-fix] 备份 %s' % os.path.relpath(backup, px4))
    open(path, 'w', encoding='utf-8').write(new_text)
    print('[groundtruth-fix] 已修补 %s（删除析构里的 world_->Reset()）' % os.path.relpath(path, px4))
    print('[groundtruth-fix] 完成。重新编译：')
    print('    cd %s/build/px4_sitl_default/build_gazebo && ninja libgazebo_groundtruth_plugin.so' % px4)
    print('[groundtruth-fix] 还原：python3 %s --revert' % os.path.abspath(__file__))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
