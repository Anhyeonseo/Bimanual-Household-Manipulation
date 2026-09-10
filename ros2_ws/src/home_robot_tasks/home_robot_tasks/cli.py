"""Print a fetch plan from local JSON files without opening any hardware."""
import argparse
import json
from pathlib import Path

from .fetch import FetchRequest, plan_fetch


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--request", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        request = FetchRequest.from_dict(json.loads(args.request.read_text(encoding="utf-8")))
        world = json.loads(args.world.read_text(encoding="utf-8"))
        plan = plan_fetch(request, world)
    except (OSError, ValueError) as error:
        parser.exit(2, f"Invalid task: {error}\n")
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
