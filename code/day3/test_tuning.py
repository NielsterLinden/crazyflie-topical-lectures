"""Asynchronous fake-parameter tests; never connect to a radio."""

from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import queue
import sys
from types import SimpleNamespace
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from race_common import tuning as backend
from . import flight
from .tuning import make_tuning


URI = "radio://0/9/2M/E7E7E7E701"
BASELINE = {"posCtlPid.xKp": 2., "posCtlPid.yKp": 2., "kalman.pNAcc_xy": .5}


def profile(mode="controller", factor=1.05):
    return dict(version=1, uri=URI, baseline=dict(BASELINE), mode=mode, factor=factor,
                factor_range=[.95, 1.05] if mode == "controller" else [.8, 1.2], range_checked=True)


class FakeParams:
    """Serialize delayed firmware reads/writes like the cflib parameter updater."""
    def __init__(self):
        self.values = dict(BASELINE, **{"stabilizer.controller": 1., "stabilizer.estimator": 2.})
        self.is_updated = True
        self.callbacks = {}
        self.toc = SimpleNamespace(get_element_by_complete_name=lambda name: object() if name in self.values else None)
        self.operations = queue.Queue()
        self.writes = []
        self.suppressed = lambda name, value: False
        self.worker = threading.Thread(target=self._work, daemon=True)
        self.worker.start()

    def add_update_callback(self, group, name, cb):
        self.callbacks[group + "." + name] = cb

    def remove_update_callback(self, group, name, cb):
        self.callbacks.pop(group + "." + name, None)

    def set_value(self, name, value):
        self.writes.append((name, float(value)))
        self.operations.put((name, float(value)))

    def request_param_update(self, name):
        self.operations.put((name, None))

    def _work(self):
        while True:
            item = self.operations.get()
            if item is None:
                return
            name, value = item
            time.sleep(.004)
            if value is not None:
                self.values[name] = value
            callback = self.callbacks.get(name)
            if callback is not None and not self.suppressed(name, self.values[name]):
                callback(name, str(self.values[name]))

    def close(self):
        self.operations.put(None)
        self.worker.join(timeout=2)
        assert not self.worker.is_alive()


class TuningTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.params = FakeParams()
        self.addCleanup(self.params.close)
        self.cf = SimpleNamespace(param=self.params)

    def test_only_one_family_and_agreed_bounded_factors_are_accepted(self):
        self.assertEqual(backend.validate_tuning(profile(), URI)["factor"], 1.05)
        for field, value in (("mode", "attitude"), ("factor", 1.2), ("factor", float("nan")),
                             ("factor_range", [.5, 2]), ("range_checked", False), ("uri", URI + "0")):
            data = profile()
            data[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                backend.validate_tuning(data, URI)
        data = profile()
        data["baseline"]["pid_attitude.roll_kp"] = 10
        with self.assertRaises(ValueError):
            backend.validate_tuning(data)

    def test_controller_and_optional_kalman_restore_delayed_writes(self):
        for mode in ("controller", "kalman"):
            with self.subTest(mode=mode):
                path = self.root / (mode + ".json")
                before = len(self.params.writes)
                with backend.temporary_tuning(self.cf, profile(mode), URI, path):
                    for name in backend.MODES[mode]:
                        self.assertAlmostEqual(self.params.values[name], BASELINE[name] * 1.05)
                    untuned = set(BASELINE) - set(backend.MODES[mode])
                    for name in untuned:
                        self.assertEqual(self.params.values[name], BASELINE[name])
                self.assertEqual({name: self.params.values[name] for name in BASELINE}, BASELINE)
                receipt = json.loads(path.read_text())
                self.assertEqual(receipt["status"], "restored")
                self.assertEqual(set(receipt["restored"]), set(backend.MODES[mode]))
                self.assertEqual(len(self.params.writes) - before, 2 * len(backend.MODES[mode]))
        self.assertFalse(self.params.callbacks)

    def test_abort_and_interrupt_restore_before_context_exits(self):
        for index, error in enumerate((RuntimeError("tracking abort"), KeyboardInterrupt())):
            path = self.root / f"abort-{index}.json"
            with self.assertRaises(type(error)):
                with backend.temporary_tuning(self.cf, profile(), URI, path):
                    raise error
            self.assertEqual(json.loads(path.read_text())["status"], "restored")
            self.assertEqual(self.params.values["posCtlPid.xKp"], 2.)

    def test_each_stretch_group_changes_only_selected_parameters_and_restores(self):
        baseline = {name: float(index + 1) for index, name in enumerate(backend.PARAMETERS)}
        self.params.values.update(baseline)
        for mode, groups in backend.PARAMETER_GROUPS.items():
            for group, names in groups.items():
                with self.subTest(mode=mode, group=group):
                    data = profile(mode)
                    data.update(baseline=baseline, parameter_group=group)
                    path = self.root / (group + ".json")
                    with backend.temporary_tuning(self.cf, data, URI, path):
                        for name, value in baseline.items():
                            expected = value * 1.05 if name in names else value
                            self.assertAlmostEqual(self.params.values[name], expected)
                    receipt = json.loads(path.read_text())
                    self.assertEqual(set(receipt["attempted"]), set(names))
                    self.assertEqual(receipt["status"], "restored")
                    self.assertEqual({name: self.params.values[name] for name in baseline}, baseline)

    def test_stretch_rejects_wrong_family_missing_zero_and_unbounded_choices(self):
        for mode, group, baseline in (
                ("controller", "acceleration_z", BASELINE),
                ("kalman", "velocity_p_xy", BASELINE),
                ("controller", "velocity_p_xy", BASELINE),
                ("kalman", "position", dict(BASELINE, **{"kalman.pNPos": 0})),
                ("controller", "attitude_p", BASELINE)):
            data = profile(mode)
            data.update(parameter_group=group, baseline=baseline)
            with self.subTest(mode=mode, group=group), self.assertRaises(ValueError):
                backend.validate_tuning(data)
        data = profile()
        data.update(parameter_group="velocity_p_xy", factor=1.11,
                    baseline=dict(BASELINE, **{"velCtlPid.vxKp": 25., "velCtlPid.vyKp": 25.}))
        with self.assertRaises(ValueError):
            backend.validate_tuning(data)
        self.assertEqual(self.params.writes, [])

    def test_capture_keeps_zero_inactive_gains_and_frontend_selects_stretch_group(self):
        baseline = dict(BASELINE, **{"posCtlPid.xKi": 0, "posCtlPid.yKi": 0,
                                    "velCtlPid.vxKp": 25., "velCtlPid.vyKp": 20.})
        path = self.root / "stretch-baseline.json"
        path.write_text(json.dumps(dict(uri=URI, parameters=baseline, motor_stop_check={"value": 1})))
        self.assertEqual(make_tuning(path)["baseline"], baseline)
        tuned = make_tuning(path, mode="controller", parameter_group="velocity_p_xy",
                            factor=1.05, factor_range=[.95, 1.05], range_checked=True)
        self.assertEqual(backend.selected_parameters(tuned), ("velCtlPid.vxKp", "velCtlPid.vyKp"))
        with self.assertRaisesRegex(ValueError, "zero baseline"):
            make_tuning(path, mode="controller", parameter_group="position_i_xy",
                        factor=1.05, factor_range=[.95, 1.05], range_checked=True)
        with self.assertRaises(ValueError):
            make_tuning(path, parameter_group="velocity_p_xy")

    def test_stretch_abort_restores_delayed_writes(self):
        data = profile("kalman")
        data.update(parameter_group="acceleration_z", baseline=dict(BASELINE, **{"kalman.pNAcc_z": 1.}))
        self.params.values.update(data["baseline"])
        with self.assertRaises(KeyboardInterrupt):
            with backend.temporary_tuning(self.cf, data, URI, self.root / "stretch-abort.json"):
                self.assertAlmostEqual(self.params.values["kalman.pNAcc_z"], 1.05)
                raise KeyboardInterrupt()
        self.assertEqual(self.params.values["kalman.pNAcc_z"], 1.)

    def test_changed_baseline_or_wrong_controller_fails_before_any_write(self):
        for name, value in (("posCtlPid.xKp", 2.2), ("stabilizer.controller", 2), ("stabilizer.estimator", 1)):
            original = self.params.values[name]
            self.params.values[name] = value
            with self.subTest(name=name), self.assertRaises(ValueError):
                with backend.temporary_tuning(self.cf, profile(), URI, self.root / (name + ".json")):
                    self.fail("must not reach flight")
            self.params.values[name] = original
        self.assertEqual(self.params.writes, [])

    def test_partial_apply_timeout_restores_unacknowledged_write(self):
        self.params.suppressed = lambda name, value: name == "posCtlPid.yKp" and value != 2
        original = backend.parameter_value
        with patch.object(backend, "parameter_value", side_effect=lambda *a, **kw: original(*a, timeout_s=.07, **kw)):
            with self.assertRaises(TimeoutError):
                with backend.temporary_tuning(self.cf, profile(), URI, self.root / "partial.json"):
                    self.fail("unacknowledged writes cannot arm")
        self.assertEqual(self.params.values["posCtlPid.xKp"], 2.)
        self.assertEqual(self.params.values["posCtlPid.yKp"], 2.)
        receipt = json.loads((self.root / "partial.json").read_text())
        self.assertEqual(receipt["status"], "restored")
        self.assertEqual(set(receipt["restored"]), set(backend.CONTROLLER_PARAMETERS))

    def test_failed_restoration_checks_remaining_parameters_and_reports_failure(self):
        original = backend.parameter_value
        path = self.root / "restore-failed.json"
        with patch.object(backend, "parameter_value", side_effect=lambda *a, **kw: original(*a, timeout_s=.07, **kw)):
            with self.assertRaisesRegex(RuntimeError, "restoration was not verified"):
                with backend.temporary_tuning(self.cf, profile(), URI, path):
                    self.params.suppressed = lambda name, value: name == "posCtlPid.yKp" and value == 2
        receipt = json.loads(path.read_text())
        self.assertEqual(receipt["status"], "restore_failed")
        self.assertIn("posCtlPid.yKp", receipt["restore_errors"])
        self.assertIn("posCtlPid.xKp", receipt["restored"])
        self.assertFalse(self.params.callbacks)

    def test_baseline_capture_file_builds_plan_without_mutating_it(self):
        path = self.root / "baseline.json"
        path.write_text(json.dumps(dict(uri=URI, parameters=BASELINE, motor_stop_check={"value": 1})))
        before = path.read_bytes()
        baseline = make_tuning(path)
        self.assertEqual(baseline["mode"], "baseline")
        self.assertEqual(backend.MODES["baseline"], ())
        tuned = make_tuning(path, mode="controller", factor=1.05, factor_range=[.95, 1.05], range_checked=True)
        self.assertEqual(tuned, profile())
        self.assertEqual(path.read_bytes(), before)

    def test_parent_rejects_missing_or_nonfinite_restoration_evidence(self):
        path = self.root / "receipt.json"
        for receipt in ({}, dict(tuning=profile(), status="restored", attempted=["posCtlPid.xKp"],
                                restored={"posCtlPid.xKp": float("nan")})):
            path.write_text(json.dumps(receipt))
            with self.assertRaisesRegex(RuntimeError, "restoration unverified"):
                flight._verify_restoration(path, profile())

    def test_notebook_capture_reads_only_under_verified_stop(self):
        from . import tuning as frontend
        context = Mock()
        context.__enter__ = Mock(return_value=(SimpleNamespace(cf=self.cf), {"value": 1}))
        context.__exit__ = Mock(return_value=False)
        path = self.root / "captured.json"
        with patch.object(frontend, "motor_stopped_connection", return_value=context), redirect_stdout(io.StringIO()):
            frontend._capture(URI, path)
        self.assertEqual(make_tuning(path)["baseline"], BASELINE)
        self.assertEqual(self.params.writes, [])
        context.__exit__.assert_called_once()

    def test_parent_binds_and_archives_tuning_and_requires_runner_support(self):
        course = self.root / "course.json"
        course.write_text(json.dumps(dict(course_name="test", enabled_for_flight=True)))
        runner = self.root / "runner.py"
        runner.write_text('''import json, sys
from pathlib import Path
if "--tuning" in sys.argv:
    tuning = json.loads(Path(sys.argv[sys.argv.index("--tuning") + 1]).read_text())
    print("TUNING PROTOCOL 1: test", flush=True)
if "--fly" in sys.argv:
    assert input("confirm: ") == "test"
    log = Path(sys.argv[sys.argv.index("--log") + 1])
    names = ["posCtlPid.xKp", "posCtlPid.yKp"]
    log.with_suffix(".parameters.json").write_text(json.dumps(dict(
        tuning=tuning, status="restored", attempted=names,
        restored={name: tuning["baseline"][name] for name in names})))
''')
        with redirect_stdout(io.StringIO()):
            notebook = json.loads(Path(__file__).with_name("03_flight.ipynb").read_text())
            source = next("".join(c["source"]) for c in notebook["cells"] if c["id"] == "flight-tool")
            values = dict(runner=runner, course_json=course, runner_python=Path(sys.executable), runner_directory=self.root)
            for name, value in values.items():
                source = source.replace(f"{name} = None", f"{name} = Path({str(value)!r})")
            source = source.replace("run_dry_check = False", "run_dry_check = True")
            scope = dict(Path=Path, output=self.root, dry_run=flight.dry_run, new_attempt=flight.new_attempt,
                         tuning_spec=profile())
            exec(source, scope)
            checked = scope["checked_run"]
            self.assertEqual(json.loads(checked.tuning_json), profile())
            archived = flight.fly(checked, URI, self.root / "race.csv", self.root / "flight.txt",
                                  ready=True, confirm=lambda _: "test", tuning=profile())
        self.assertEqual(archived.read_bytes(), course.read_bytes())
        self.assertEqual(json.loads((self.root / "race.tuning.json").read_text()), profile())
        changed = profile(factor=1.02)
        with patch.object(flight, "_run") as launch:
            with self.assertRaisesRegex(ValueError, "Tuning changed"):
                flight.fly(checked, URI, self.root / "changed.csv", self.root / "changed.txt",
                           ready=True, tuning=changed)
            launch.assert_not_called()
        runner.write_text("print('old runner')\n")
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "did not confirm tuning support"):
            flight.dry_run(runner, course, sys.executable, self.root, self.root / "old.txt", tuning=profile())


if __name__ == "__main__":
    unittest.main()
