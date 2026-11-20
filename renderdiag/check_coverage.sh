#!/bin/sh
# check_coverage.sh: 列出四个渲染模块中未被 renderdiag 映射表覆盖的失败点。
#
# 失败点 = 错误哨兵返回, 匹配:
#   return -1            哨兵返回
#   return;              void 函数提前返回 (静默丢弃)
#   printf.*return 1;    demo 打印错误后中止
#   return rc;           携带哨兵值的返回
# 覆盖 = 失败点同行或前 3 行内出现 RD_REPORT( 或 rd-waive: 注释。
# 另校验: 模块引用的 RD_* 码必须存在于映射表; 表中未被引用的项打印提示。
#
# 退出码: 0 = 全部覆盖; 1 = 有未覆盖失败点或未映射错误码 (已列出)。
set -u
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
header="$root/renderdiag/src/renderdiag.h"
modules="particle/src/particle.c tilemap/src/tilemap.c navgrid/src/navgrid.c spatial/src/spatial.c"

rc=0

echo "== 未覆盖的失败点 =="
uncovered=$(
for m in $modules; do
    awk '
        {
            lines[NR] = $0
            if ($0 ~ /return -1|return;|printf.*return 1;|return rc;/) {
                covered = 0
                for (i = NR - 3; i <= NR; i++)
                    if (lines[i] ~ /RD_REPORT\(|rd-waive:/) covered = 1
                if (!covered) printf "%s:%d: %s\n", FILENAME, NR, $0
            }
        }
    ' "$root/$m"
done
)
if [ -n "$uncovered" ]; then
    echo "$uncovered"
    rc=1
else
    echo "(无)"
fi

echo "== 模块引用但映射表中没有的错误码 =="
defined=$(grep -oE 'RD_[A-Z_0-9]+' "$header" | sort -u)
used=$(for m in $modules; do grep -hoE 'RD_[A-Z_0-9]+' "$root/$m"; done | sort -u)
missing=$(echo "$used" | grep -Fvx "$defined" || true)
if [ -n "$missing" ]; then
    echo "$missing"
    rc=1
else
    echo "(无)"
fi

echo "== 映射表中未被任何失败点引用的项 (提示, 不影响结果) =="
table_codes=$(sed -n '/rd_table\[\]/,/^};/p' "$header" | grep -oE 'RD_[A-Z_0-9]+' | sort -u)
refs=$( { for m in $modules; do grep -hoE 'RD_[A-Z_0-9]+' "$root/$m"; done; \
          grep -oE 'RD_[A-Z_0-9]+' "$root/renderdiag/src/renderdiag.c"; } | sort -u )
unused=$(echo "$table_codes" | grep -Fvx "$refs" || true)
if [ -n "$unused" ]; then echo "$unused"; else echo "(无)"; fi

if [ "$rc" -eq 0 ]; then
    echo "coverage_ok=1"
else
    echo "coverage_ok=0"
fi
exit "$rc"
