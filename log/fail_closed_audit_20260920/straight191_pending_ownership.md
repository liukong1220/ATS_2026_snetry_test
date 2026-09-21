# Straight191 — cold-start prior + pending ownership

Date: 2026-09-21

## DoD cell
- MuJoCo straight goal-set with `GOAL_SET_VERIFY_FREE=1` (0.42 m), DualMap auth, action SUCCEEDED.

## Root causes closed
1. **Install stale lost-timeout (189):** `install/.../rmuc_2025_mujoco.launch.py` still had `observation_lost_timeout_s=10.0` while source had 30.0. Synced; source remains authoritative.
2. **Cold-start false minima (189):** lattice accepted yaw≈−2.3 before first `STATUS_ACCEPTED`, poisoning free-space. Added `cold_start_prior_max_xy_m` / `cold_start_prior_max_yaw_rad` gate before confirmation.
3. **Pending wiped after leaving LOST (190):** pending opened under LOST; fusion → RELOCALIZING made `preferMultiGuess()` false; `fine_only` reject called `clearConfirmation()`. Fix: pending forces multi_guess path; rejects while pending retain the hypothesis. Latch `pending_holds_lattice_origin_` **before** installing pending so UNINITIALIZED adopt semantics stay intact.

## Verification
- Focused gtests: ColdStartPrior, PendingOwns, ConfirmationRetains, ColdLost/Bootstrap, PostAcceptLost — PASS.
- `/tmp/ats_goal_set_straight_191_final/summary.json`: **passed**; action/safety/recorder/teardown all passed.
- Evidence: free-space OK; `emit mode=1`; `auto_authorized=1`; action `SUCCEEDED`; `final_distance≈0.012 m`; contact_violation_delta=0 (physical contact evaluator still unverified).
- Prior: MuJoCo real GICP recovery185 PASS (do not overwrite).

## Not claimed
- Four-environment DoD (Gazebo/real HIL still open at write time).
- Offline ATS replay; 100-repeat; soak; content-lineage atomic snapshot.
