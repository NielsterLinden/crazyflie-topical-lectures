# Race day, drone 4 (5 Nov 2026)

## Race 1: slow 150 s plan with stops, organisers' format
- Plan: `race2/plan_cons/trajectory.csv`, submission `submissions/team-hoop-storm/v01/` (baseline + optimized tuning.json, xKp/yKp x0.95).
- Full battery (>= 4.10 V on the charger). Reboot the drone first so nothing from race 2 tests is left set.
- Own laptop fallback: `cd code/day1 && PY rehearse.py fly ../../race2/plan_cons/trajectory.csv absolute=1 vbat_min=4.0`

## Race 2: the plan is `race2/plan_fast/trajectory.csv` (sha256 50ad2715306d..., 36.6 s, 0.7 m/s, no stops)
Flown clean 14:08 (`code/day1/data/flip_4_1408_rehearsal.csv`): all 7 hoop passes, no fence.
Regenerate exactly: `PY race2/fast_path.py` (defaults v_max=0.7 v_hoop=0.5 a=0.9 lead=0.5).

    cd code/day1
    PY rehearse.py fly ../../race2/plan_fast/trajectory.csv absolute=1 lead=0.5 hold=0.5 vbat_min=3.85 lh_std=0.0001 arm=$S/GO
    # drone next to (0,0,0), facing +x; ARMED -> place on (0,0,0) -> go -> touch $S/GO

`lh_std=0.0001` (Lighthouse trust x4) is what made it work: the height sensor reads the hoop 3 bottom rim and the
hoop 1 rim (range.zrange 0.3-0.5 m) and drags the estimate down. A reboot clears it.
