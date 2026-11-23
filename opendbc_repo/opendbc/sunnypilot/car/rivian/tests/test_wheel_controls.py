"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.sunnypilot.car.rivian.carstate_ext import (
  CarStateExt,
  FAST_SCROLL_DETENTS,
  LONG_PRESS_FRAMES,
  SCROLL_BURST_GAP_FRAMES,
  SCROLL_MAX_STEP,
  SCROLL_UP_SIGN,
  WheelSettings,
)
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

GAP = (ButtonType.gapAdjustCruise, False)


# Built without __init__, which reads openpilot Params; only the wheel state is needed here.
def _make(gen2=False, custom=False, short=1, long=5, metric=False):
  CP = structs.CarParams(brand='rivian', openpilotLongitudinalControl=True)
  CP_SP = structs.CarParamsSP()
  CP_SP.flags |= RivianFlagsSP.LONGITUDINAL_HARNESS_UPGRADE.value
  cs = CarStateExt.__new__(CarStateExt)
  cs.CP, cs.CP_SP = CP, CP_SP
  cs.set_speed = 10
  cs.increase_button = False
  cs.decrease_button = False
  cs.distance_button = 0
  cs.increase_counter = 0
  cs.decrease_counter = 0
  cs.vdm_user_adas_request = 0
  cs._lkas_pending = False
  cs.acc_fault_recorder = SimpleNamespace(update=lambda *a: None)
  cs._resume_enabled = False
  cs.last_active_set_speed = None
  cs._prev_cruise_enabled = False
  cs._resume_eligible = False
  cs._resume_acc_counter = 0
  cs._prev_stalk_down2 = False
  cs._prev_stalk_down = False
  cs._frames_since_acc_on = 0
  cs._init_wheel(WheelSettings(gen2, custom, short, long))
  cs.set_speed = 40 * CV.MPH_TO_MS
  return cs


class _Driver:
  def __init__(self, cs, metric=False):
    self.cs = cs
    self.parsers = {
      Bus.pt: SimpleNamespace(vl={"VDM_AdasSts": {"VDM_UserAdasRequest": 0}}),
      Bus.alt: SimpleNamespace(vl={
        "WheelButtons_Fwd": {"RightButton_Scroll": 255,
                             "RightButton_LeftClick": 1,
                             "RightButton_RightClick": 1},
        "BSM_BlindSpotIndicator_Fwd": {"BSM_BlindSpotIndicator_Left": 0,
                                       "BSM_BlindSpotIndicator_Right": 0},
      }),
      Bus.adas: SimpleNamespace(vl={"Cluster": {"Cluster_Unit": 0 if metric else 1}}),
    }

  def frame(self, scroll=255, left=1, right=1):
    wb = self.parsers[Bus.alt].vl["WheelButtons_Fwd"]
    wb["RightButton_Scroll"] = scroll
    wb["RightButton_LeftClick"] = left
    wb["RightButton_RightClick"] = right
    ret = structs.CarState()
    ret.cruiseState.enabled = True
    ret.vEgoCluster = 20.0
    events = self.cs.update_longitudinal_upgrade(ret, self.parsers)
    return [(e.type, e.pressed) for e in events]

  @property
  def speed_mph(self):
    return self.cs.set_speed * CV.MS_TO_MPH

  @property
  def speed_kph(self):
    return self.cs.set_speed * CV.MS_TO_KPH


class TestDefaultWheel:
  def test_button_tap_steps_by_one(self):
    d = _Driver(_make())
    assert d.frame(right=2) == []
    assert d.speed_mph == pytest.approx(41)
    assert d.frame() == []
    assert d.frame(left=2) == []
    assert d.speed_mph == pytest.approx(40)
    assert d.frame() == []

  def test_hold_repeats_long_step(self):
    d = _Driver(_make())
    d.cs.set_speed = 41 * CV.MPH_TO_MS
    for _ in range(LONG_PRESS_FRAMES):
      d.frame(right=2)
    assert d.speed_mph == pytest.approx(45)
    for _ in range(LONG_PRESS_FRAMES):
      d.frame(right=2)
    assert d.speed_mph == pytest.approx(50)

  def test_scroll_emits_one_event_per_change(self):
    d = _Driver(_make())
    assert d.frame(scroll=255) == []
    assert d.frame(scroll=100) == [GAP]
    assert d.frame(scroll=100) == []
    assert d.frame(scroll=101) == [GAP]
    assert d.frame(scroll=255) == []
    assert d.speed_mph == pytest.approx(40)

  def test_metric_hold_jumps_by_10_kph(self):
    d = _Driver(_make(metric=True), metric=True)
    for _ in range(LONG_PRESS_FRAMES):
      d.frame(right=2)
    assert d.speed_kph == pytest.approx(70)

  def test_custom_increments(self):
    d = _Driver(_make(custom=True, short=2, long=10))
    assert d.frame(right=2) == []
    assert d.speed_mph == pytest.approx(42)
    d.cs.set_speed = 41 * CV.MPH_TO_MS
    for _ in range(LONG_PRESS_FRAMES):
      d.frame(right=2)
    assert d.speed_mph == pytest.approx(50)


class TestGen2Wheel:
  def test_button_release_emits_one_event(self):
    d = _Driver(_make(gen2=True))
    assert d.frame(right=2) == []
    for _ in range(199):
      assert d.frame(right=2) == []
    assert d.frame() == [GAP]
    assert d.speed_mph == pytest.approx(40)
    assert d.frame(left=2) == []
    assert d.frame(left=2) == []
    assert d.frame() == [GAP]
    assert d.speed_mph == pytest.approx(40)

  def test_single_detent_steps_by_one(self):
    d = _Driver(_make(gen2=True))
    assert d.frame(scroll=100) == []
    assert d.frame(scroll=100 + SCROLL_UP_SIGN) == []
    assert d.speed_mph == pytest.approx(41)
    assert d.frame(scroll=100) == []
    assert d.speed_mph == pytest.approx(40)

  def test_wraparound(self):
    d = _Driver(_make(gen2=True))
    assert d.frame(scroll=0) == []
    assert d.frame(scroll=255) == []
    assert d.speed_mph == pytest.approx(39)
    assert d.frame(scroll=0) == []
    assert d.speed_mph == pytest.approx(40)

  def test_slow_scroll_all_single_steps(self):
    d = _Driver(_make(gen2=True))
    v = 100
    assert d.frame(scroll=v) == []
    for _ in range(5):
      for _ in range(SCROLL_BURST_GAP_FRAMES + 10):
        d.frame(scroll=v)
      v += SCROLL_UP_SIGN
      d.frame(scroll=v)
    assert d.speed_mph == pytest.approx(45)

  def test_fast_scroll_long_steps(self):
    d = _Driver(_make(gen2=True))
    d.frame(scroll=100)
    d.cs.set_speed = 41 * CV.MPH_TO_MS
    v = 100
    for i in range(1, 2 * FAST_SCROLL_DETENTS + 1):
      v += SCROLL_UP_SIGN
      d.frame(scroll=v)
      if i == FAST_SCROLL_DETENTS:
        assert d.speed_mph == pytest.approx(45)
      elif i == 2 * FAST_SCROLL_DETENTS:
        assert d.speed_mph == pytest.approx(50)

  def test_large_jump_ignored_and_first_frame_neutral(self):
    d = _Driver(_make(gen2=True))
    # the first frame just records the baseline, it never steps
    assert d.frame(scroll=100) == []
    assert d.speed_mph == pytest.approx(40)
    # a jump beyond SCROLL_MAX_STEP in one frame is a bad frame: ignored
    assert d.frame(scroll=100 + SCROLL_MAX_STEP + 1) == []
    assert d.speed_mph == pytest.approx(40)
    # subsequent steps are measured against the new value
    assert d.frame(scroll=100 + SCROLL_MAX_STEP + 1 + SCROLL_UP_SIGN) == []
    assert d.speed_mph == pytest.approx(41)

  def test_custom_increments(self):
    d = _Driver(_make(gen2=True, custom=True, short=2, long=10))
    assert d.frame(scroll=100) == []
    assert d.frame(scroll=100 + SCROLL_UP_SIGN) == []
    assert d.speed_mph == pytest.approx(42)

  def test_speed_clamped(self):
    d = _Driver(_make(gen2=True))
    d.frame(scroll=100)
    d.cs.set_speed = 21 * CV.MPH_TO_MS
    v = 100
    for _ in range(3):
      v -= SCROLL_UP_SIGN
      d.frame(scroll=v)
    assert d.speed_mph == pytest.approx(20)

    d = _Driver(_make(gen2=True))
    d.frame(scroll=100)
    d.cs.set_speed = 84 * CV.MPH_TO_MS
    v = 100
    for _ in range(3):
      v += SCROLL_UP_SIGN
      d.frame(scroll=v)
    assert d.speed_mph == pytest.approx(85)

  def test_replay_of_the_2026_11_23_capture(self):
    """The owner's R1T capture, as received at 20 Hz (time s, RightButton_Scroll): six slow detents up, six down,
    then quick flicks up and down. Slow detents are single steps; each flick is one long-press jump."""
    capture = [(48.679, 1), (49.830, 2), (51.027, 3), (56.629, 4), (60.078, 5), (62.778, 6),
               (66.528, 5), (69.021, 4), (71.419, 3), (75.180, 2), (76.419, 1), (77.420, 0),
               (79.521, 2), (80.579, 4), (80.616, 5), (81.478, 7), (81.518, 8),
               (83.274, 5), (83.918, 3), (83.968, 2), (84.579, 255)]
    d = _Driver(_make(gen2=True))
    d.frame(scroll=0)
    seen = {}
    t, value = 48.0, 0
    for when, new in capture:
      while t < when:          # 100 Hz carstate frames, holding the last received value
        d.frame(scroll=value)
        t += 0.01
      value = new
      d.frame(scroll=value)
      t += 0.01
      seen[when] = round(d.speed_mph, 3)
    assert seen[62.778] == pytest.approx(46)   # +6 single steps from 40
    assert seen[77.420] == pytest.approx(40)   # -6 single steps
    assert seen[79.521] == pytest.approx(45)   # flick up (2 detents in one message): 41, then jump to 45
    assert seen[80.616] == pytest.approx(50)   # 46, jump to 50; the 3rd detent of the flick adds nothing
    assert seen[81.518] == pytest.approx(55)
    assert seen[83.274] == pytest.approx(50)   # flick down (3 detents at once): 54, jump to 50
    assert seen[83.968] == pytest.approx(45)
    assert seen[84.579] == pytest.approx(40)   # across the 2 -> 255 wrap: 44, jump to 40
