"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from cereal import custom
from openpilot.sunnypilot.mapd.lib.road_type_classifier import classify_road_type

RoadType = custom.LiveMapDataSP.RoadType

# Helpers — speed in m/s
MPH_TO_MS = 0.44704


class TestInterstateDetection:
  @pytest.mark.parametrize("road_name", [
    "Interstate 5",
    "Interstate 405",
    "I-5",
    "I-95",
    "I-405",
    "I-10",
  ])
  def test_interstate_names(self, road_name):
    assert classify_road_type(road_name, 65 * MPH_TO_MS) == RoadType.interstate

  def test_interstate_no_speed_limit(self):
    assert classify_road_type("I-5", 0) == RoadType.interstate

  def test_interstate_low_speed_limit(self):
    """Interstate in construction zone still classified as interstate."""
    assert classify_road_type("Interstate 5", 35 * MPH_TO_MS) == RoadType.interstate

  def test_freeway_keyword(self):
    assert classify_road_type("Santa Monica Freeway", 60 * MPH_TO_MS) == RoadType.interstate

  def test_expressway_keyword(self):
    assert classify_road_type("Long Island Expressway", 55 * MPH_TO_MS) == RoadType.interstate


class TestHighwayDetection:
  @pytest.mark.parametrize("road_name", [
    "US Route 101",
    "US-1",
    "US Highway 20",
    "U.S. 30",
  ])
  def test_us_highway_names(self, road_name):
    assert classify_road_type(road_name, 55 * MPH_TO_MS) == RoadType.highway

  @pytest.mark.parametrize("road_name", [
    "State Route 1",
    "SR-99",
    "CA-1",
    "TX-130",
    "NY-17",
  ])
  def test_state_route_high_speed(self, road_name):
    assert classify_road_type(road_name, 55 * MPH_TO_MS) == RoadType.highway

  def test_unnamed_high_speed(self):
    """High speed limit without a name pattern -> highway."""
    assert classify_road_type("", 65 * MPH_TO_MS) == RoadType.highway

  def test_highway_name_no_speed(self):
    """Named highway with no speed data -> highway."""
    assert classify_road_type("US-101", 0) == RoadType.highway


class TestUrbanDetection:
  @pytest.mark.parametrize("road_name,speed_limit_mph", [
    ("Main Street", 35),
    ("Oak Avenue", 25),
    ("5th Street", 30),
    ("Broadway", 40),
  ])
  def test_city_street_names(self, road_name, speed_limit_mph):
    assert classify_road_type(road_name, speed_limit_mph * MPH_TO_MS) == RoadType.urban

  def test_state_route_low_speed(self):
    """State route through a town at low speed -> urban."""
    assert classify_road_type("CA-1", 30 * MPH_TO_MS) == RoadType.urban

  def test_unnamed_low_speed(self):
    """Low speed, no name -> urban."""
    assert classify_road_type("", 25 * MPH_TO_MS) == RoadType.urban

  def test_named_road_no_speed(self):
    """Named road with no speed data and no highway pattern -> urban."""
    assert classify_road_type("Elm Boulevard", 0) == RoadType.urban


class TestUnknown:
  def test_no_data(self):
    assert classify_road_type("", 0) == RoadType.unknown

  def test_empty_name_zero_speed(self):
    assert classify_road_type("", 0.0) == RoadType.unknown

  def test_ambiguous_speed_no_name(self):
    """50 mph unnamed road — between urban ceiling and highway threshold."""
    assert classify_road_type("", 50 * MPH_TO_MS) == RoadType.unknown
