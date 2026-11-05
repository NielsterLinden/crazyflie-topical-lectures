"""Build submissions/team-TEAMNAME/v01/{baseline,optimized} from race2/plan_cons via the course API.

No radio. Usage (from team-crazyflie/, PY = course venv python):
  $PY submissions/make_v01.py                          # plan files only (course, summary, trajectory)
  $PY submissions/make_v01.py BASELINE.json            # + tuning.json, optimized = baseline
  $PY submissions/make_v01.py BASELINE.json 1.1        # + optimized posCtlPid.xKp/yKp * 1.1 (range [1, 1.1])
"""
from pathlib import Path
import json, shutil, sys, tempfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "code"))
from day3.notebook_tools import load_plan, save_plan
from day3.tuning import make_tuning

source = root / "race2/plan_cons/course.json"
v01 = root / "submissions/team-TEAMNAME/v01"
baseline = Path(sys.argv[1]) if len(sys.argv) > 1 else None
factor = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
plan = load_plan(source)  # checks course/summary match and recomputes the trajectory

if baseline is not None and factor != 1:
    # Same route as 03_flight.ipynb "Export the two race submissions".
    with tempfile.TemporaryDirectory() as scratch:
        pair = plan.configure_race(baseline=baseline, controller={"position_p_xy": factor},
                                   factor_range=[min(1, factor), max(1, factor)], range_checked=True
                                   ).export(Path(scratch) / "race")
        for name, paths in pair.items():
            shutil.copytree(paths["course"].parent, v01 / name, dirs_exist_ok=True)
else:
    # export_race_pair rejects factor 1 for "optimized", so an identical pair is written directly
    # with the same functions it uses (save_plan + make_tuning, same JSON layout).
    tuning = None if baseline is None else make_tuning(baseline)
    for name in ("baseline", "optimized"):
        save_plan(plan, v01 / name)
        if tuning is not None:
            (v01 / name / "tuning.json").write_text(json.dumps(tuning, indent=2, allow_nan=False) + "\n",
                                                    encoding="utf-8")
print("Wrote", v01, "| tuning:", "TODO (no baseline)" if baseline is None else f"factor {factor:g}")
