"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase

from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP
from openpilot.sunnypilot.selfdrive.controls.lib.acceleration_control import AccelerationControl
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

PERIOD = int(PARAMS_UPDATE_PERIOD / DT_MDL)


class TestAccelBoostInterlock(OpenpilotTestCase):
  """Which boost LongitudinalPlannerSP.update selects, from the real params and both real controllers."""

  def _set(self, boost, acceleration_control):
    params = Params()
    params.put_bool("AccelBoostSP", boost)
    params.put_bool("AccelerationControl", acceleration_control)

  def _planner(self, openpilot_longitudinal=True):
    CP = SimpleNamespace(openpilotLongitudinalControl=openpilot_longitudinal)
    return SimpleNamespace(accel_boost=AccelBoost(), accel_boost_sp=AccelBoostSP(), acceleration_control=AccelerationControl(CP),
                           openpilot_longitudinal=openpilot_longitudinal, events_sp=SimpleNamespace(clear=lambda: None),
                           e2e_alerts_helper=SimpleNamespace(update=lambda sm, events: None))

  def _run(self, planner, frames=PERIOD + 1):
    for _ in range(frames):
      LongitudinalPlannerSP.update(planner, {})

  def test_boost_runs_with_acceleration_control_off(self):
    self._set(boost=True, acceleration_control=False)
    planner = self._planner()
    self._run(planner)
    assert planner.accel_boost is planner.accel_boost_sp

  def test_boost_off_with_acceleration_control_on(self):
    self._set(boost=True, acceleration_control=True)
    planner = self._planner()
    self._run(planner)
    assert type(planner.accel_boost) is AccelBoost

  def test_comma_boost_is_not_rebuilt_every_frame(self):
    # it learns from overrides, so a new instance each frame would silently lose that
    self._set(boost=True, acceleration_control=True)
    planner = self._planner()
    self._run(planner, 1)
    kept = planner.accel_boost
    self._run(planner, 3 * PERIOD)
    assert planner.accel_boost is kept

  def test_toggling_boost_with_acceleration_control_on_never_selects_it(self):
    self._set(boost=True, acceleration_control=True)
    planner = self._planner()
    for boost in (False, True, False, True):
      Params().put_bool("AccelBoostSP", boost)
      self._run(planner, 2 * PERIOD)
      assert type(planner.accel_boost) is AccelBoost

  def test_acceleration_control_turned_on_after_start_takes_boost_off(self):
    self._set(boost=True, acceleration_control=False)
    planner = self._planner()
    self._run(planner)
    assert planner.accel_boost is planner.accel_boost_sp
    Params().put_bool("AccelerationControl", True)
    planner.acceleration_control._refresh()
    self._run(planner, 1)
    assert type(planner.accel_boost) is AccelBoost

  def test_both_off_keeps_comma_boost(self):
    self._set(boost=False, acceleration_control=False)
    planner = self._planner()
    self._run(planner)
    assert type(planner.accel_boost) is AccelBoost

  def test_no_openpilot_longitudinal_keeps_comma_boost(self):
    self._set(boost=True, acceleration_control=False)
    planner = self._planner(openpilot_longitudinal=False)
    self._run(planner)
    assert type(planner.accel_boost) is AccelBoost
