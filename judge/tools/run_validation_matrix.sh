#!/bin/bash
# 验证矩阵：按评审要求的顺序跑，逐例产出 ROUND 判据 + 关键证据
#
# 用例（base0 本身不含 rover，"20 rover"只在随机图上出现，故矩阵按 3 例组织）：
#   A base0            集成基线（链路：清理→仿真→六机→坐标→裁判→/score）
#   B random rovers=0  随机地图本身（随机建筑/人物 + 六机链路）
#   C random rovers=20 完整负载（含 rover）
#
# 用法：bash tools/run_validation_matrix.sh [round-limit=75] [drones=6]
set -u
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIMIT="${1:-75}"; DRONES="${2:-6}"
FAILED_CASES=()
OUT="$WORKSPACE_DIR/logs/referee_round/matrix_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

run_case() {
    local name="$1"; shift
    echo "=============================================================="
    echo ">>> 用例 $name: $*"
    echo "=============================================================="
    bash "$WORKSPACE_DIR/start_referee_round.sh" "$@" --round-limit="$LIMIT" \
        >"$OUT/${name}.log" 2>&1
    local rc=$?
    local dir
    dir="$(sed -n 's/.*证据目录: *//p' "$OUT/${name}.log" | tail -1)"
    local verdict
    verdict="$(grep -aE '^  判据:' "$OUT/${name}.log" | tail -1 | sed 's/^ *//')"
    local score
    score="$(grep -aE '^  裁判 /score:' "$OUT/${name}.log" | tail -1 | sed 's/^ *//')"
    if [ "$rc" != "0" ] || ! printf '%s' "$verdict" | grep -q "ROUND PASS"; then
        # 用例失败：rc 非 0，或判据不是 ROUND PASS（防止 start_referee_round.sh 某天又变成
        # "失败也返回 0"）。矩阵继续跑完（要收集三例结果），但最后整体返回非 0。
        FAILED_CASES+=("$name(rc=$rc,${verdict:-无判据})")
    fi
    printf '%s|rc=%s|%s|%s|%s\n' "$name" "$rc" "$verdict" "$score" "$dir" >>"$OUT/summary.txt"
}

run_case "A_base0"          --world=base0  --display=headless --drones="$DRONES"
run_case "B_random_r0"      --world=random --display=headless --drones="$DRONES" --rovers=0
run_case "C_random_r20"     --world=random --display=headless --drones="$DRONES" --rovers=20

echo
echo "======================= 矩阵汇总 ======================="
column -s'|' -t "$OUT/summary.txt" 2>/dev/null || cat "$OUT/summary.txt"
echo
echo "逐例详情：$OUT/<用例>.log ；每例证据目录见上表最后一列"
echo
if [ ${#FAILED_CASES[@]} -gt 0 ]; then
    echo "=================== 矩阵结论：存在失败用例 ==================="
    for case in "${FAILED_CASES[@]}"; do echo "  FAILED: $case"; done
    echo "矩阵整体退出码 = 1（CI/自动测试可直接依赖）"
    exit 1
fi
echo "=================== 矩阵结论：三例全部 ROUND PASS ==================="
echo "矩阵整体退出码 = 0"
exit 0
