#!/usr/bin/env bash
# =============================================================================
# 任务1/任务2 静态契约的"变异测试"（2026-09-28）
#
#   做法：把 src/{tracking,look_up,yolo} 复制到 /tmp 副本，在副本上逐个注入
#   "退化解"，确认静态契约测试会失败（= 断言真的在守着这些性质）。
#   基线（未变异）必须通过。
#
#   A 租约默认 20 → 0（= 永不过期，回到"锁死不放"）          → look_up + tracking 测试都要失败
#   B 去掉抢占路径（租约过期分支恒假，请求永远被拒）          → look_up + tracking 测试都要失败
#   C 抢占默认 true → false（一键回退成"永不抢占"）           → look_up + tracking 测试都要失败
#   D 去掉"真框年龄 > synthetic_only_release_seconds"判据     → tracking 测试失败
#   E 强制归还时写 completed_target_ids_（污染后续判定）      → tracking 测试失败
#   F 去掉 tracker 的锁心跳调用（不续租，20s 后被抢走）        → tracking 测试失败
#
# 运行：bash src/tracking/test/lock_task_mutation_test.sh
# =============================================================================
set -u
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"   # = <repo>/src
BASE=$(mktemp -d /tmp/lock_task_mut.XXXXXX)
cp -r "$SRC/tracking" "$SRC/look_up" "$SRC/yolo" "$BASE/"
LOOKUP_TEST="$BASE/look_up/test/lock_lease_static_test.py"
TRACK_TEST="$BASE/tracking/test/return_policy_static_test.py"
LOOKUP_CPP="$BASE/look_up/src/target_lookup_service.cpp"
LOOKUP_HDR="$BASE/look_up/include/look_up/target_lookup_service.h"
TRACK_CPP="$BASE/tracking/src/state_machine.cpp"

run_case () {   # run_case <期望: pass|fail> <标签> <测试文件>
  local expect="$1" name="$2" test_file="$3"
  local out; out=$(python3 "$test_file" 2>&1); local rc=$?
  local last; last=$(printf '%s\n' "$out" | grep -E "AssertionError|Error" | tail -1)
  [ -z "$last" ] && last=$(printf '%s\n' "$out" | tail -1)
  if [ "$expect" = fail ]; then
    if [ $rc -ne 0 ]; then printf '  [CAUGHT]  %-22s exit=%d  %s\n' "$name" "$rc" "$last"
    else printf '  [MISSED!] %-22s exit=%d  ← 变异没被抓住！\n' "$name" "$rc"; fi
  else
    if [ $rc -eq 0 ]; then printf '  [PASS]    %-22s exit=0\n' "$name"
    else printf '  [BROKEN!] %-22s exit=%d  %s\n' "$name" "$rc" "$last"; fi
  fi
}

echo "仓库副本: $BASE"
echo "基线（未变异）："
run_case pass "baseline look_up" "$LOOKUP_TEST"
run_case pass "baseline tracking" "$TRACK_TEST"
echo

echo "变异 A：look_up 租约默认 20 → 0（= 永不过期）"
python3 - "$LOOKUP_CPP" "$LOOKUP_HDR" <<'PY'
import sys, pathlib
cpp, hdr = (pathlib.Path(p) for p in sys.argv[1:3])
s = cpp.read_text(); old = 'pnh_.param("lock_lease_seconds", lock_lease_seconds_, 20.0);'
assert old in s, "A: cpp 锚点缺失"
cpp.write_text(s.replace(old, 'pnh_.param("lock_lease_seconds", lock_lease_seconds_, 0.0);'))
h = hdr.read_text(); anchor = "double lock_lease_seconds_ = 20.0;"
assert anchor in h, "A: hdr 锚点缺失"
hdr.write_text(h.replace(anchor, "double lock_lease_seconds_ = 0.0;"))
PY
run_case fail "A look_up(默认0)" "$LOOKUP_TEST"
run_case fail "A tracking(默认0)" "$TRACK_TEST"
cp "$SRC/look_up/src/target_lookup_service.cpp" "$LOOKUP_CPP"
cp "$SRC/look_up/include/look_up/target_lookup_service.h" "$LOOKUP_HDR"
echo

echo "变异 B：去掉抢占（过期分支恒假 → 请求永远被拒）"
python3 - "$LOOKUP_CPP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
old = "if (leaseExpiredLocked(target_id, now)) {"
assert old in s, "B: 锚点缺失"
p.write_text(s.replace(old, "if (false) {  // MUTATION B: 去掉抢占"))
PY
run_case fail "B look_up(无抢占)" "$LOOKUP_TEST"
run_case fail "B tracking(无抢占)" "$TRACK_TEST"
cp "$SRC/look_up/src/target_lookup_service.cpp" "$LOOKUP_CPP"
echo

echo "变异 C：抢占开关默认 true → false（永不抢占）"
python3 - "$LOOKUP_CPP" "$LOOKUP_HDR" <<'PY'
import sys, pathlib
cpp, hdr = (pathlib.Path(p) for p in sys.argv[1:3])
s = cpp.read_text(); old = 'pnh_.param("lock_preempt_enabled", lock_preempt_enabled_, true);'
assert old in s, "C: cpp 锚点缺失"
cpp.write_text(s.replace(old, 'pnh_.param("lock_preempt_enabled", lock_preempt_enabled_, false);'))
h = hdr.read_text(); anchor = "bool lock_preempt_enabled_ = true;"
assert anchor in h, "C: hdr 锚点缺失"
hdr.write_text(h.replace(anchor, "bool lock_preempt_enabled_ = false;"))
PY
run_case fail "C look_up(禁抢占)" "$LOOKUP_TEST"
run_case fail "C tracking(禁抢占)" "$TRACK_TEST"
cp "$SRC/look_up/src/target_lookup_service.cpp" "$LOOKUP_CPP"
cp "$SRC/look_up/include/look_up/target_lookup_service.h" "$LOOKUP_HDR"
echo

echo "变异 D：去掉'真框年龄 > synthetic_only_release_seconds'判据"
python3 - "$TRACK_CPP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
old = "    if (real_box_age_seconds <= synthetic_only_release_seconds_) return false;"
assert old in s, "D: 锚点缺失"
p.write_text(s.replace(old, "    // MUTATION D: 去掉真框年龄判据（只看 90s 就归还）"))
PY
run_case fail "D tracking(无5s判据)" "$TRACK_TEST"
cp "$SRC/tracking/src/state_machine.cpp" "$TRACK_CPP"
echo

echo "变异 E：强制归还时写 completed_target_ids_（污染后续判定）"
python3 - "$TRACK_CPP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
anchor = ("                force_non_elimination_return = true;\n"
          "            } else if (suppressNonEliminationReturn(")
assert anchor in s, "E: 锚点缺失"
p.write_text(s.replace(anchor,
    "                completed_target_ids_.insert(currently_tracked_target_id_);  // MUTATION E\n" + anchor, 1))
PY
run_case fail "E tracking(写完成标记)" "$TRACK_TEST"
cp "$SRC/tracking/src/state_machine.cpp" "$TRACK_CPP"
echo

echo "变异 F：去掉 tracker 的锁心跳调用（不续租 → 租约到期被抢走）"
python3 - "$TRACK_CPP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
old = "    heartbeatTargetLock();\n"
assert old in s, "F: 锚点缺失"
p.write_text(s.replace(old, "    // MUTATION F: 不发心跳\n", 1))
PY
run_case fail "F tracking(无心跳)" "$TRACK_TEST"
echo
echo "副本保留在 $BASE（可 rm -rf 删除）"
