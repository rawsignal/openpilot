# dev-a mirror test run — 58affa715d (PR #10: sunnypilot 6f7a2c94c sync + Acceleration Control)

Date: 2026-11-03
Branch: `qwen/deva-mirror-tests` (WSL worktree `/home/logan/qwen-deva-tests`, shared `.git` at `E:\AdventurePilot-Dev`)
Purpose: run the four test targets that require the compiled `msgq` bindings (and `libparams_c.so`), which had
never been run on this commit. No production code was modified.

**TL;DR — all four targets now run on a plain venv + scons-built msgq/params. 41 passed, 2 failed, 1 skipped,
plus 1 SCC test file that cannot even be collected (stale import in the test itself). Both failures are
test-side bugs in this commit, not environment gaps and not production-code bugs.**

| # | target | result |
|---|--------|--------|
| 1 | `openpilot/sunnypilot/selfdrive/controls/lib/tests/test_acceleration_control_planner.py` | **8 passed, 2 failed** |
| 2 | `openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/tests/` | **0 run — 1 collection error** (test_curve_speed_controller.py; map 5 passed + vision 7 passed when run per-file) |
| 3 | `openpilot/sunnypilot/selfdrive/car/tests/test_car_specific_sp.py` | **3 passed** |
| 4 | `openpilot/sunnypilot/sunnylink/tests/test_settings_changes.py` | **18 passed, 1 skipped** |

---

## 1. Git state (before / after)

HEAD (identical before and after):

```
58affa715df17cbea473e2c0346e0e8ef6c311bb
```

Gitlinks — `git ls-tree HEAD msgq_repo panda rednose_repo teleoprtc_repo tinygrad_repo openpilot/sunnypilot/neural_network_data`:

| gitlink | before | after |
|---------|--------|-------|
| `msgq_repo` | `e7396e76dadbb49e374d4b664ff6bbb43a39bcb0` | `e7396e76dadbb49e374d4b664ff6bbb43a39bcb0` |
| `panda` | `8a16e089b88593914823743df29e1c30b7dd3e00` | `8a16e089b88593914823743df29e1c30b7dd3e00` |
| `rednose_repo` | `28d4a7f69e80e1c3e0d24ca0733d7daeaeade3d0` | `28d4a7f69e80e1c3e0d24ca0733d7daeaeade3d0` |
| `teleoprtc_repo` | `1aa8fc433bef1519a95c0700c96258c3be6dfb34` | `1aa8fc433bef1519a95c0700c96258c3be6dfb34` |
| `tinygrad_repo` | `92b742f711c7d216fd3ed7a46a30a959e3c43bd4` | `92b742f711c7d216fd3ed7a46a30a959e3c43bd4` |
| `openpilot/sunnypilot/neural_network_data` | `03cac2d30e111e0689c0429cb8c1fe6cb5a905af` | `03cac2d30e111e0689c0429cb8c1fe6cb5a905af` |

Every gitlink is unchanged. `git status --porcelain` in the worktree is empty after all builds (build
artifacts are gitignored, in the main tree and inside the initialized submodules).
Submodules left intentionally uninitialized: `teleoprtc_repo`, `tinygrad_repo`, `openpilot/sunnypilot/neural_network_data`.

## 2. Commands run, in order (including failed attempts)

Windows PowerShell unless noted; WSL = `wsl -d Ubuntu -- bash -lc ...` (non-trivial bash was put in LF script
files under `C:\Users\logan\AppData\Local\Temp\opencode\` because PowerShell mangles `2>&1`/pipes/`$` inside
`wsl -- bash -c "..."`).

1. `git -C E:\AdventurePilot-Dev fetch https://github.com/rawsignal/openpilot.git main`
   → OK. (PowerShell prints the stderr banner `From https://github.com/rawsignal/openpilot` as a
   `RemoteException` and reports exit code 1 — cosmetic, FETCH_HEAD updated; the commit
   `58affa715d` was already in the object store via `deva-pr10`.)
2. `wsl ... "cd /mnt/e/AdventurePilot-Dev && GIT_LFS_SKIP_SMUDGE=1 git worktree add -b qwen/deva-mirror-tests /home/logan/qwen-deva-tests 58affa715df17cbea473e2c0346e0e8ef6c311bb"`
   → OK, 2218 files.
3. `git rev-parse HEAD` + `git ls-tree HEAD <6 gitlinks>` (the "before" values in §1) → recorded.
4. Read-only exploration: `SConstruct` (all of it), every SConscript that gets parsed, `pyproject.toml`,
   `uv.lock`, `openpilot/common/params.py`, `openpilot/cereal/messaging/__init__.py`,
   `msgq_repo/msgq/__init__.py`, the four test files, `lateral_load_governor.py`, `longitudinal_planner.py`.
   → Found: SConstruct imports 8 comma-deps Python modules at parse time; `panda/SConscript`,
   `rednose_repo/.../SConscript`, cabana/jotpluggler SConscripts need `opendbc`, `bootstrap_icons`, `imgui`,
   `libusb` modules at parse time; `params.cc` includes cereal's *generated* capnp headers, so cereal codegen
   is in the dependency graph of `libparams_c.so`; `import msgq` needs `msgq_repo` on sys.path.
5. `GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --force msgq_repo rednose_repo panda`
   → OK. All three checked out at exactly the gitlink commits from §1.
   (msgq_repo = build; rednose_repo + panda = required just to *parse* the SConstruct/SConscripts.
   teleoprtc_repo/tinygrad_repo/neural_network_data left uninitialized — see §6.)
6. `uv venv /home/logan/qwen-venv --python 3.12` (uv 0.9.26) → created venv on CPython 3.12.12.
7. `VIRTUAL_ENV=/home/logan/qwen-venv uv sync` → **FAILED**: lock requires Python 3.12.13;
   `uv python install 3.12.13` → **FAILED** on uv 0.9.26 ("No download found for
   cpython-3.12.13-linux-x86_64-gnu").
8. `uv self update` → 0.9.26 → 0.13.7 (user-level `~/.local`). `uv python install 3.12.13` → OK (managed
   CPython under `~/.local/share/uv/python`).
9. `uv venv /home/logan/qwen-venv --python 3.12.13` (recreated the venv) → OK.
10. `VIRTUAL_ENV=/home/logan/qwen-venv uv sync --active` → **FAILED**: `Failed to generate package metadata
    for teleoprtc==1.0.1 @ editable+teleoprtc_repo ... does not appear to be a Python project`.
    `pyproject.toml` `[tool.uv] default-groups = ["standalone"]` makes `uv sync` install all six submodules as
    editable sources (incl. uninitialized teleoprtc_repo, tinygrad_repo). **Abandoned `uv sync`** to avoid
    initializing two more submodules; used surgical `uv pip install` instead (venv is outside the repo, so
    nothing in the worktree is affected).
11. `uv pip install` batch 1 (build deps): `scons cython "numpy>=2.0" "pycapnp==2.1.0" pyzmq setproctitle
    requests tqdm comma-deps-acados comma-deps-capnproto comma-deps-eigen comma-deps-ffmpeg
    comma-deps-json11 comma-deps-ncurses comma-deps-zeromq comma-deps-zstd` → OK.
    (pycapnp pinned to 2.1.0 per pyproject: "2.2 introduces a memory leak".)
12. `uv pip install` batch 2 (test deps): `pytest pytest-xdist hypothesis pycryptodome pyserial pyyaml ruff
    "PyJWT[crypto]" websocket_client jeepney zstandard pillow` → OK.
13. Wrote venv path shim `/home/logan/qwen-venv/lib/python3.12/site-packages/qwen_msgq_repo.pth` containing
    `/home/logan/qwen-deva-tests/msgq_repo` → makes `import msgq` resolve (the task's PYTHONPATH
    `.../qwen-deva-tests:.../opendbc_repo` does not include `msgq_repo`, where the `msgq` package lives).
    This is venv-local; no repo file touched.
14. `scons -Q openpilot/common/libparams_c.so msgq_repo/libmsgq.so msgq_repo/msgq/ipc_pyx.so` → **FAILED**
    (two distinct problems):
    a. `ModuleNotFoundError: No module named 'bootstrap_icons'` in `openpilot/tools/cabana/SConscript`
       (parse-time import; jotpluggler SConscript needs the same three modules).
    b. `scons: *** Do not know how to make File target 'msgq_repo/libmsgq.so'` — wrong target name.
15. `uv pip install comma-deps-bootstrap-icons comma-deps-imgui comma-deps-libusb` → OK (build asked for them).
16. Retried scons with `--tree=prune` to find real targets → discovered `env.Library('msgq')` is **static**
    (`msgq_repo/libmsgq.a`) and that `params.o` depends on `openpilot/cereal/gen/cpp/*.capnp.h` (capnpc
    codegen is in the graph). → **FAILED** (three problems):
    a. Cap'n Proto "Version mismatch between generated code and library headers": `capnpc` resolved from
       `~/.local/bin/capnpc` (v1.3.0, a pre-existing standalone install) generated code against
       comma-deps-capnproto **1.0.1** headers.
    b. `sh: 1: cythonize: not found` — venv bin not on PATH.
    c. (hidden) the same capnp mismatch as the real compile error for `params.o`/`swaglog.o`.
17. Fixed both: `export PATH="/home/logan/qwen-venv/bin:$PATH"` (venv first, like the prebuilt CI workflow
    does) and `rm -f openpilot/cereal/gen/cpp/*.c++ openpilot/cereal/gen/cpp/*.h` (stale 1.3.0-generated
    files). Retried → **FAILED**: `cythonize` crashed with `ModuleNotFoundError: No module named 'distutils'`
    (Python 3.12 removed distutils; Cython 3.3.1's cythonize imports `distutils.extension`).
18. `uv pip install setuptools` (84.2.1 — provides the `distutils` shim) → OK.
19. `PATH=/home/logan/qwen-venv/bin:$PATH scons -Q openpilot/common/libparams_c.so msgq_repo/libmsgq.a msgq_repo/msgq/ipc_pyx.so`
    → **OK (exit 0)**. Artifacts: `openpilot/common/libparams_c.so` (3.9 MB),
    `msgq_repo/libmsgq.a` (1.4 MB, linked into ipc_pyx.so), `msgq_repo/msgq/ipc_pyx.so` (1.6 MB).
20. Import smoke test (PYTHONPATH as in the task + venv): `import msgq` → OK;
    `from openpilot.common.params import Params; Params()` → OK (loads libparams_c.so);
    `from openpilot.cereal import messaging` → OK; `from openpilot.common.test import OpenpilotTestCase` → OK.
21. First pytest pass, all four targets with the exact task command (below). → T1 collection error
    (`No module named '...longitudinal_mpc_lib.c_generated_code'`), T2 collection error
    (`cannot import name 'A_LAT_CEILING'`), T3 3 passed, T4 18 passed 1 skipped.
22. `scons -Q openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so`
    (venv PATH) → **OK (exit 0)**. Ran acados codegen (`python3 long_mpc.py`) and built
    `c_generated_code/libacados_ocp_solver_long.so` + `c_generated_code/acados_ocp_solver_pyx.so` (plus copied
    `libacados.so`/`libblasfeo.so`/`libhpipm.so` into `c_generated_code/`).
23. Re-ran T1 → **8 passed, 2 failed** (failures analyzed in §4).
24. Ran the three SCC test files individually (supplementary — the directory target aborts on the first
    collection error) → curve_speed: collection error (0 run), map: 5 passed, vision: 7 passed.
25. Final official pass of all four targets with the exact task command → results in §4 (worktree still
    `git status`-clean afterwards).
26. `git rev-parse HEAD` + `git ls-tree HEAD <6 gitlinks>` (the "after" values in §1) → unchanged.
27. Wrote this report; `git add reports/deva-mirror-test-run.md`; `git status --porcelain` +
    `git diff --cached --stat`; committed in WSL; pushed from Windows git.

The exact test command used for every target (run from `/home/logan/qwen-deva-tests`):

```
PYTHONPATH=/home/logan/qwen-deva-tests:/home/logan/qwen-deva-tests/opendbc_repo \
  /home/logan/qwen-venv/bin/python -m pytest -q -p no:cacheprovider <target>
```

## 3. Reproducible build recipe (the part that worked)

```bash
# 0) Prereqs present on this machine (NOT installed by this run — no apt packages were installed):
#    build-essential, libzmq3-dev, python3-dev, zlib1g-dev, system scons (unused; venv scons is used)

# 1) Fresh WSL worktree at the target commit
git -C E:/AdventurePilot-Dev fetch https://github.com/rawsignal/openpilot.git main
wsl -d Ubuntu -- bash -lc "cd /mnt/e/AdventurePilot-Dev && GIT_LFS_SKIP_SMUDGE=1 git worktree add \
  -b qwen/deva-mirror-tests /home/logan/qwen-deva-tests 58affa715df17cbea473e2c0346e0e8ef6c311bb"
cd /home/logan/qwen-deva-tests

# 2) Submodules: msgq (build) + rednose + panda (required only to PARSE the SConstruct/SConscripts)
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --force msgq_repo rednose_repo panda

# 3) Fresh venv OUTSIDE the repo, Python 3.12.13 (uv.lock pins that exact interpreter)
uv self-update-check && uv --version   # needs >= ~0.13 for a 3.12.13 download; 0.9.26 cannot fetch it
uv venv /home/logan/qwen-venv --python 3.12.13
P=/home/logan/qwen-venv/bin
VIRTUAL_ENV=/home/logan/qwen-venv uv pip install --python $P/python \
  scons cython "numpy>=2.0" "pycapnp==2.1.0" pyzmq setproctitle requests tqdm \
  comma-deps-acados comma-deps-capnproto comma-deps-eigen comma-deps-ffmpeg \
  comma-deps-json11 comma-deps-ncurses comma-deps-zeromq comma-deps-zstd \
  comma-deps-bootstrap-icons comma-deps-imgui comma-deps-libusb \
  pytest pytest-xdist hypothesis pycryptodome pyserial pyyaml ruff \
  "PyJWT[crypto]" websocket_client jeepney zstandard pillow setuptools
# setuptools: Python 3.12 has no distutils; scons' cython tool shells out to `cythonize`, which imports it.

# 4) Make `import msgq` resolvable (msgq package lives in msgq_repo/msgq/, not on the task PYTHONPATH):
echo /home/logan/qwen-deva-tests/msgq_repo > /home/logan/qwen-venv/lib/python3.12/site-packages/qwen_msgq_repo.pth

# 5) Build ONLY what the tests need. PATH with the venv FIRST is mandatory:
#    - `capnpc`/`cythonize`/`python3` must come from the venv (comma-deps capnproto 1.0.1, not the
#      pre-existing ~/.local/bin/capnpc 1.3.0 — that mix fails with a capnp version-mismatch #error)
#    - the cereal gen step (capnpc) is a hard dependency of libparams_c.so because params.cc
#      includes openpilot/cereal/gen/cpp/{log,custom,deprecated,car}.capnp.h
export PATH=/home/logan/qwen-venv/bin:$PATH
scons -Q openpilot/common/libparams_c.so msgq_repo/libmsgq.a msgq_repo/msgq/ipc_pyx.so
# target 1 additionally needs the generated acados MPC solver (long_mpc.py codegen + cython):
scons -Q openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so

# 6) Run tests (from the worktree root)
export PYTHONPATH=/home/logan/qwen-deva-tests:/home/logan/qwen-deva-tests/opendbc_repo
/home/logan/qwen-venv/bin/python -m pytest -q -p no:cacheprovider \
  openpilot/sunnypilot/selfdrive/controls/lib/tests/test_acceleration_control_planner.py
# ... (same pattern for the other three targets)
```

Notes:
- `msgq` is built as a **static** `libmsgq.a` in this tree; the Python binding is `msgq_repo/msgq/ipc_pyx.so`
  (Cython), with the msgq objects linked in. `libparams_c.so` lands next to `openpilot/common/params.py`,
  where `params.py` dlopens it (`Path(__file__).with_name("libparams_c.so")`).
- If you ever regenerate `openpilot/cereal/gen/cpp/` with a foreign capnpc, delete the stale
  `*.c++`/`*.h` before rebuilding (scons keys the generation command on the capnpc binary node, but a
  half-updated tree is confusing).
- Full build deliberately NOT done (no loggerd/modeld/UI/panda-firmware/etc.).

## 4. Test results (final official pass) and full failure output

### Target 1 — `openpilot/sunnypilot/selfdrive/controls/lib/tests/test_acceleration_control_planner.py`

**8 passed, 2 failed, 0 skipped** (exit 1).

(The first run of this target failed at *collection* with `ModuleNotFoundError: No module named
'openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code'` — the generated acados MPC solver
was not built yet. After step 22 above, it runs. That gap is an environment/build gap, not a code bug.)

Full output of the two failures (identical except the parametrized values; complete below):

```
___________ test_parent_to_sp_requested_telemetry_and_disable[0.1-1] ___________

monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x7f30c177f980>
cruise = 0.1, lead = 1

    @pytest.mark.parametrize('cruise,lead', [(0.1, 1), (1, 0.05)])
    def test_parent_to_sp_requested_telemetry_and_disable(monkeypatch, cruise, lead):
      planner, sm = _planner(monkeypatch, 0.1, cruise, lead)
      planner.update(sm)
      # Real capnp vCruise is float32; its kph -> m/s conversion has sub-micro precision.
      assert planner.output_a_target <= 0.1 + 1e-6
      _publication_fields(planner)
      pm = Publisher()
      planner.publish(sm, pm)
      shared = pm.messages['longitudinalPlan']
      fork = pm.messages['longitudinalPlanSP']
      assert 'accelerationControlBoost' not in shared
      assert fork['accelerationControlBoostRequested'] == pytest.approx(0.2)
      assert fork['accelerationControlBoostRequested'] > shared['aTarget']
      # Same object, positive preceding request: disabled update must publish zero.
      sm['selfdriveState'].enabled = False
>     planner.update(sm)

openpilot/sunnypilot/selfdrive/controls/lib/tests/test_acceleration_control_planner.py:143:
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

self = <openpilot.selfdrive.controls.lib.longitudinal_planner.LongitudinalPlanner object at 0x7f30c182c9b0>
sm = {'carState': <car.capnp:CarState builder (deprecated = (gas = 0, brake = 0, brakeLights = false, clutchPressed = false...eadK = 0, aLeadK = 0, present = true, aLeadTau = 0, modelProb = 0, radar = false, radarTrackId = -1), mdMonoTime = 0)>}

    def update(self, sm):
      LongitudinalPlannerSP.update(self, sm)

      v_ego = sm['carState'].vEgo
      v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
      v_cruise = v_cruise_kph * CV.KPH_TO_MS
      if sm['controlsState'].forceDecel:
        v_cruise = 0.0

      long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

      # Reset current state when not engaged, or user is controlling the speed
      reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      # PCM cruise speed may be updated a few cycles later, check if initialized
      v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
      reset_state = reset_state or not v_cruise_initialized

      if reset_state:
        self.v_desired_filter.x = v_ego
        self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
        self.a_cruise = self.output_a_target

      # Prevent divergence, smooth in current v_ego
      self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

      # No change cost when user is controlling the speed, or when standstill
      prev_accel_constraint = not (reset_state or sm['carState'].standstill)

      # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
      v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

      self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
      if self.CP.openpilotLongitudinalControl:
        self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
      else:
        self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
      self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

      self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
      self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
      self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

      # TODO counter is only needed because radar is glitchy, remove once radar is gone
      self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
      if self.fcw:
        cloudlog.info("FCW triggered")

      # Save starting point for next iteration
      a_prev = self.output_a_target

      action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
      output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                action_t=action_t)
      output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
      output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
      output_should_stop_e2e = sm['modelV2'].action.shouldStop

      self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.dt)
      cruise_should_stop = should_stop(v_ego, self.a_cruise)

      # Acceleration Control first: Gas Override Boost latches "the model is limiting" from the candidate it is
      # given, so it has to see the lifted one or it learns a boost from overrides where the lead was the limit.
>     output_a_target_e2e = self.acceleration_control.update(sm, output_a_target_e2e, self.scc.governor.throttle_scale())
                                                                                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
E     AttributeError: 'types.SimpleNamespace' object has no attribute 'throttle_scale'

openpilot/selfdrive/controls/lib/longitudinal_planner.py:116: AttributeError
```

`test_parent_to_sp_requested_telemetry_and_disable[1-0.05]` is byte-identical except `cruise = 1, lead = 0.05`
and the object addresses.

```
=========================== short test summary info ============================
FAILED openpilot/sunnypilot/selfdrive/controls/lib/tests/test_acceleration_control_planner.py::test_parent_to_sp_requested_telemetry_and_disable[0.1-1]
FAILED openpilot/sunnypilot/selfdrive/controls/lib/tests/test_acceleration_control_planner.py::test_parent_to_sp_requested_telemetry_and_disable[1-0.05]
2 failed, 8 passed in 0.42s
```

**One-line diagnosis:** the test's own `_publication_fields()` helper (line ~120 of
`test_acceleration_control_planner.py`) swaps `planner.scc` for a `SimpleNamespace` whose
`governor=SimpleNamespace(load=0, ceiling=1, is_active=False)` lacks `throttle_scale`, and the test then calls
`planner.update()` a second time, which hits the PR #10 production line
`self.scc.governor.throttle_scale()` (`longitudinal_planner.py:116`).
**Classification: real repo bug, but test-side (stale/incomplete fake), NOT a production-code bug and NOT an
environment gap** — the production `LateralLoadGovernor` defines `throttle_scale()`
(`lateral_load_governor.py:109`), so the real on-road object has the attribute; the fixture just wasn't
updated when Acceleration Control started calling it.

### Target 2 — `openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/tests/`

**0 run, 1 collection error** (exit 2) — pytest aborts the whole directory on the first collection error.

Full output:

```
==================================== ERRORS ====================================
_ ERROR collecting openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/tests/test_curve_speed_controller.py _
ImportError while importing test module '/home/logan/qwen-deva-tests/openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/tests/test_curve_speed_controller.py'.
Hint: make sure your test modules/packages have valid Python names.
Traceback:
../.local/share/uv/python/cpython-3.12.13-linux-x86_64-gnu/lib/python3.12/importlib/__init__.py:90: in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/tests/test_curve_speed_controller.py:16: in <module>
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.lateral_load_governor import (
E   ImportError: cannot import name 'A_LAT_CEILING' from 'openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.lateral_load_governor' (/home/logan/qwen-deva-tests/openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/lateral_load_governor.py)
=========================== short test summary info ============================
ERROR openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/tests/test_curve_speed_controller.py
!!!!!!!!!!!!!!!!!!!! Interrupted: 1 error during collection !!!!!!!!!!!!!!!!!!!!
1 error in 0.27s
```

**One-line diagnosis:** `test_curve_speed_controller.py:16` imports `A_LAT_CEILING` from
`lateral_load_governor`, which only defines `A_LAT_CEILING_TORQUE` (line 36) and takes the angle-mode ceiling
from `MAX_LATERAL_ACCEL_NO_ROLL`; the bare name `A_LAT_CEILING` exists nowhere in production code in this
commit (the governor file itself is fork-original — it is absent from `sunnypilot/master`).
**Classification: real repo bug, test-side (stale reference to a renamed constant), NOT an environment gap.**
The whole SCC directory is blocked by this one import.

Supplementary per-file runs (same command pattern; the directory target itself aborts before running any of
them, so these are extra data, not a different "target"):

```
test_curve_speed_controller.py : ERROR (same ImportError), 0 run
test_map_controller.py         : 5 passed
test_vision_controller.py      : 7 passed
```

### Target 3 — `openpilot/sunnypilot/selfdrive/car/tests/test_car_specific_sp.py`

**3 passed, 0 failed, 0 skipped** (exit 0).

```
...                                                                      [100%]
3 passed in 0.17s
```

### Target 4 — `openpilot/sunnypilot/sunnylink/tests/test_settings_changes.py`

**18 passed, 0 failed, 1 skipped** (exit 0).

```
.........s.........                                                      [100%]
18 passed, 1 skipped in 0.26s
```

(The skip is in-test, platform-conditional; not investigated further — it's a pass-state.)

### Totals

| passed | failed | skipped | collection errors (tests not run) |
|--------|--------|---------|-----------------------------------|
| 41 | 2 | 1 | 1 file (test_curve_speed_controller.py, 0 tests executed) |

## 5. What was NOT run, and why

- **Full scons build** (loggerd, camerad, modeld, locationd, UI, panda firmware, cabana, etc.) — intentionally
  skipped; only the minimal targets the four test files need were built (§3).
- **`uv sync`** (the repo's canonical environment, installs all six submodules as editable packages per
  `[tool.uv] default-groups = ["standalone"]`) — NOT RUN: it requires `teleoprtc_repo` and `tinygrad_repo`
  to be initialized as editable Python sources, which the "initialize only what the build needs" constraint
  avoids. Surgical `uv pip install` (versions from the same PyPI wheels uv.lock pins) was used instead; the
  venv is outside the repo so nothing in the worktree depends on the abandoned path.
- **Everything else in the repo** (other SP tests, opendbc safety/car suites, `test_models`, replay tools) —
  out of scope for this task.
- **On-device / CI execution** — not applicable here.

## 6. Submodules

Initialized (all at the exact gitlink commits from §1, verified with `git submodule status`):
`msgq_repo` (build), `rednose_repo` + `panda` (parse-time requirements of the SConstruct/SConscripts).
Left uninitialized: `teleoprtc_repo`, `tinygrad_repo`, `openpilot/sunnypilot/neural_network_data` (nothing in
the four targets or the minimal build needs them; see §5 re: `uv sync`).

## 7. Environment changes made by this run

- **apt packages installed: none.** (build-essential, libzmq3-dev, python3-dev, zlib1g-dev, system scons
  were already present; the system scons was not used — the venv's scons 4.11.1 is.)
- `uv` self-updated 0.9.26 → 0.13.7 (user-level, `~/.local`) — required to fetch CPython 3.12.13.
- uv-managed CPython 3.12.13 installed (`~/.local/share/uv/python`) — the exact interpreter `uv.lock` pins.
- Fresh venv `/home/logan/qwen-venv` (outside the repo). Installed: scons 4.11.1, cython 3.3.1, numpy 2.5.4,
  pycapnp 2.1.0, pyzmq 27.2.0, setproctitle 1.3.8, requests 2.34.2, tqdm 4.70.1, setuptools 84.2.1,
  pytest 9.2.0, pytest-xdist 3.8.0, hypothesis 6.170.14, pycryptodome 4.3.0, pyserial 3.5, pyyaml 6.0.3,
  ruff 0.17.3, PyJWT 2.15.1 (+cryptography), websocket-client 1.9.2, jeepney 0.9.0, zstandard 0.25.0,
  pillow 13.0.0, comma-deps-{acados 0.2.2.post106, capnproto 1.0.1, eigen 3.4.0.post106, ffmpeg
  7.1.0.post106, json11 20170411.0.post106, ncurses 6.5.post106, zeromq 4.3.5.post103, zstd 1.5.6.post106,
  bootstrap-icons 1.10.5.0.post106, imgui 1.92.7.post106, libusb 1.0.29.post106} (+ their transitive deps).
- Venv-local shim: `site-packages/qwen_msgq_repo.pth` → `/home/logan/qwen-deva-tests/msgq_repo` (see step 13).
- Build artifacts left in the worktree (all gitignored; `git status --porcelain` is empty):
  `openpilot/common/libcommon.a`, `openpilot/common/libparams_c.so`, `openpilot/cereal/gen/cpp/*`,
  `msgq_repo/libmsgq.a`, `msgq_repo/msgq/ipc_pyx.so`, `openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/*`.

## 8. Source changes that seemed necessary but were NOT made

None. No tracked file in the worktree was edited. (If someone wants to fix the two test bugs: add
`throttle_scale=lambda: 1.0` — or a real governor — to the `scc.governor` fake in
`_publication_fields()` in `test_acceleration_control_planner.py`, and import `A_LAT_CEILING_TORQUE` instead
of `A_LAT_CEILING` in `test_curve_speed_controller.py` (adjusting the one use site). Both are test-file-only
changes; no production code needs to move.)
