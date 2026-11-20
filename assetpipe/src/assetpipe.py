#!/usr/bin/env python3
"""assetpipe 命令行入口。

用法:
  python3 -I assetpipe.py --plan plan.json --source assets --out build [--only id ...]
退出码: 0 成功；1 资源/预算错误（明细带文件名与档位）；2 plan 配置错误。
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import builder
from pipecommon import PipeError
import plan as plan_mod


def main(argv=None):
    ap = argparse.ArgumentParser(description="确定性资源构建流水线")
    ap.add_argument("--plan", required=True, help="构建计划 JSON")
    ap.add_argument("--source", required=True, help="源资源目录（只读）")
    ap.add_argument("--out", required=True, help="构建输出目录")
    ap.add_argument("--only", nargs="*", default=None,
                    help="只构建指定 target id")
    args = ap.parse_args(argv)

    try:
        cfg = plan_mod.load_plan(args.plan)
        manifest, errors = builder.run_pipeline(
            cfg, args.source, args.out, only_targets=args.only)
    except PipeError as e:
        print("配置错误: %s" % e, file=sys.stderr)
        return 2

    n_targets = len(manifest["targets"])
    n_files = sum(len(t["textures"]) + len(t["audios"])
                  + len(t["atlases"]) for t in manifest["targets"])
    print("assetpipe: 构建 %d 个目标，产出 %d 个资源条目，manifest=%s"
          % (n_targets, n_files,
             os.path.join(os.path.abspath(args.out), builder.MANIFEST_NAME)))
    for t in manifest["targets"]:
        b = t["budgets"]["actual"]
        print("  - %s (%s/%s): 纹理 %d B, 音频 %d B, 图集 %d B, 合计 %d B"
              % (t["target"], t["platform"], t["quality"],
                 b["textures"], b["audios"], b["atlases"], b["total"]))

    if errors:
        print("\n构建失败，共 %d 个错误:" % len(errors), file=sys.stderr)
        for e in errors:
            print("  ! %s" % e, file=sys.stderr)
        return 1
    print("全部目标构建成功，预算均未超支。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
