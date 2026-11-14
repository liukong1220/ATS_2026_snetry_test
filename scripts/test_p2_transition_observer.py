from copy import deepcopy
from p2_transition_observer import analyze


def evidence():
    events = [{"kind": "phase", "phase": p, "mono": t} for p, t in
              [("fault", 1), ("stopped", 2), ("recovery_requested", 5),
               ("recovered", 6), ("complete", 12)]]
    events += [{"kind": "reference", "mono": .3, "stamp_ns": 100},
               {"kind": "execution", "mono": .4, "mode": 1},
               {"kind": "selected", "mono": .5, "magnitude": .3},
               {"kind": "emergency_stop", "mono": 1.4, "value": True},
               {"kind": "ready", "mono": 7, "value": True}]
    for t in range(2, 13):
        for name in ("autonomy_raw", "selected", "motion_control"):
            events.append({"kind": name, "mono": t, "magnitude": 0.0, "ros_ns": 100})
    return sorted(events, key=lambda e: e["mono"])


def test_frozen_ros_clock_still_measures_recovery_duration():
    result = analyze(evidence())
    assert result["passed"]
    assert result["recovery_duration_sec_monotonic"] == 6


def test_late_old_reference_or_retimed_reference_fails_even_with_zero_commands():
    for stamp in (100, 200):
        events = evidence() + [{"kind": "reference", "mono": 8, "stamp_ns": stamp}]
        assert not analyze(events)["passed"]


def test_mpc_nonzero_cannot_be_hidden_by_zero_arbitration():
    events = evidence()
    events.append({"kind": "autonomy_raw", "mono": 8.5, "magnitude": .02})
    assert not analyze(events)["passed"]


def test_missing_samples_are_not_zero_motion_evidence():
    events = [e for e in evidence() if not (e["kind"] == "motion_control" and e["mono"] > 7)]
    assert not analyze(events)["passed"]


def test_incomplete_run_fails():
    events = deepcopy(evidence())
    events = [e for e in events if e.get("phase") != "complete"]
    assert not analyze(events)["passed"]


def test_revival_before_recovery_ready_is_not_hidden_by_the_final_zero_window():
    events = evidence() + [{"kind": "reference", "mono": 5.1, "stamp_ns": 100},
                           {"kind": "autonomy_raw", "mono": 5.2, "magnitude": .3}]
    result = analyze(events)
    assert not result['passed']
    assert not result['windows']['recovering']['autonomy_raw']['passed']
    assert result['recovered_old_reference_count'] == 1
