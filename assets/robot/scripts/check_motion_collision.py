"""Read-only local collision review of an existing package; never runs hardware."""
import argparse
import json
from pathlib import Path

from collision_checker import DEFAULT_RULES
from motion_collision import check_package, save_draft_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    parser.add_argument("--stage", choices=("quick", "candidate", "trajectory"), default="trajectory")
    parser.add_argument("--time", type=float, help="Seconds within the motion, for quick/candidate review")
    parser.add_argument("--rules", type=Path, help="Override package collision rules (otherwise package policy or default)")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.stage == "trajectory" and args.time is not None:
        parser.error("--time applies to quick/candidate checks")
    report = check_package(args.package, rules_path=args.rules, stage=args.stage, at_seconds=args.time)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        path = args.report
    else:
        path = save_draft_report(report, args.package.name)
    print(json.dumps(dict(stage=report["stage"], status=report["status"],
        report=str(path.resolve()), can_export=report.get("can_export", False),
        checked_poses=report.get("checked_poses"), mesh_queries=report.get("mesh_queries"),
        findings=report.get("findings", [])[:5]), ensure_ascii=False, indent=2))
    return 0 if report["status"] in ("passed", "clear_bounds", "needs_mesh") else 2 if report["status"] == "failed" else 3


if __name__ == "__main__":
    raise SystemExit(main())
