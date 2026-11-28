"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import get_lat_smooth_seconds
from openpilot.sunnypilot.modeld_v2.modeld import ModelState


@pytest.mark.parametrize('v_ego, expected', [(0.0, 0.4), (2.0, 0.4), (5.0, 0.2), (8.0, 0.0), (30.0, 0.0)])
def test_schedule(v_ego, expected):
  assert get_lat_smooth_seconds(v_ego, 0.4) == pytest.approx(expected)


def test_schedule_is_off_for_cars_without_it():
  assert all(get_lat_smooth_seconds(v, 0.0) == 0.0 for v in (0.0, 1.0, 5.0, 20.0))


def _state(generation=12, lat=0.0):
  return SimpleNamespace(generation=generation, LAT_SMOOTH_SECONDS=lat, LONG_SMOOTH_SECONDS=0.3,
                         MIN_LAT_CONTROL_SPEED=0.3, PLANPLUS_CONTROL=1.0, mlsim=False, constants=None)


def _action(state, v_ego, lat_smooth_seconds=None, raw_curvature=0.01):
  # the 'action' output path: desired_curvature = action[0, 0] / max(1, v)^2
  output = {'action': np.array([[raw_curvature * max(1.0, v_ego) ** 2, 0.0]], dtype=np.float32)}
  prev = log.ModelDataV2.Action(desiredCurvature=0.0, desiredAcceleration=0.0)
  return ModelState.get_action_from_model(state, output, prev, 0.3, 0.3, v_ego, None, lat_smooth_seconds=lat_smooth_seconds)


def test_crawl_curvature_is_smoothed_by_the_passed_time():
  alpha = 1 - math.exp(-DT_MDL / 0.4)
  assert _action(_state(), 1.0, lat_smooth_seconds=0.4).desiredCurvature == pytest.approx(0.01 * alpha, rel=1e-4)


def test_without_a_passed_time_the_models_own_override_applies():
  assert _action(_state(lat=0.0), 1.0).desiredCurvature == pytest.approx(0.01, rel=1e-4)
  alpha = 1 - math.exp(-DT_MDL / 0.2)
  assert _action(_state(lat=0.2), 1.0).desiredCurvature == pytest.approx(0.01 * alpha, rel=1e-4)


def test_zero_time_passes_the_model_through():
  assert _action(_state(), 10.0, lat_smooth_seconds=0.0).desiredCurvature == pytest.approx(0.01, rel=1e-4)


def test_pre_fof_models_are_left_alone():
  assert _action(_state(generation=9), 1.0, lat_smooth_seconds=0.4).desiredCurvature == pytest.approx(0.01, rel=1e-4)
