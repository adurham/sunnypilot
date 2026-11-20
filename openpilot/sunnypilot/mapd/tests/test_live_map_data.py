"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from unittest.mock import MagicMock, patch

from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import custom

RoadType = custom.LiveMapDataSP.RoadType

MPH_TO_MS = 0.44704


class TestOsmMapDataRoadType(OpenpilotTestCase):
  """Verify OsmMapData.get_current_road_type() wires params through the classifier."""

  def _make(self, road_name, speed_limit):
    with patch('openpilot.sunnypilot.mapd.live_map_data.osm_map_data.BaseMapData.__init__'):
      from openpilot.sunnypilot.mapd.live_map_data.osm_map_data import OsmMapData
      obj = OsmMapData.__new__(OsmMapData)
      obj.mem_params = MagicMock()
      obj.mem_params.get.side_effect = lambda key: {
        "RoadName": road_name,
        "MapSpeedLimit": speed_limit,
      }.get(key)
      return obj

  def test_interstate_from_params(self):
    obj = self._make("Interstate 5", "29.1")
    self.assertEqual(obj.get_current_road_type(), RoadType.interstate)

  def test_highway_from_params(self):
    obj = self._make("US-101", str(55 * MPH_TO_MS))
    self.assertEqual(obj.get_current_road_type(), RoadType.highway)

  def test_urban_from_params(self):
    obj = self._make("Main Street", str(35 * MPH_TO_MS))
    self.assertEqual(obj.get_current_road_type(), RoadType.urban)

  def test_unknown_when_no_data(self):
    obj = self._make(None, None)
    self.assertEqual(obj.get_current_road_type(), RoadType.unknown)


def _fake_map_data(road_name, road_type, gps_ok):
  from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData

  class FakeMapData(BaseMapData):
    def __init__(self):
      self.sm = MagicMock()
      self.pm = MagicMock()

    def update_location(self):
      pass

    def get_current_speed_limit(self):
      return 29.1

    def get_next_speed_limit_and_distance(self):
      return 0.0, 0.0

    def get_current_road_name(self):
      return road_name

    def get_current_road_type(self):
      return road_type

  fake = FakeMapData()
  fake.sm['liveLocationKalman'].gpsOK = gps_ok
  return fake


class TestBaseMapDataPublish(OpenpilotTestCase):
  """Verify publish() sets roadType on the cereal message."""

  def _published(self, fake):
    sent_messages = []
    fake.pm.send.side_effect = lambda topic, msg: sent_messages.append((topic, msg))
    fake.publish()
    return sent_messages

  def test_publish_includes_road_type(self):
    fake = _fake_map_data("Interstate 5", RoadType.interstate, True)
    sent_messages = self._published(fake)

    self.assertEqual(len(sent_messages), 1)
    topic, msg = sent_messages[0]
    self.assertEqual(topic, 'liveMapDataSP')
    self.assertEqual(msg.liveMapDataSP.roadType, RoadType.interstate)
    self.assertEqual(msg.liveMapDataSP.roadName, "Interstate 5")

  def test_publish_road_type_urban(self):
    fake = _fake_map_data("Elm Street", RoadType.urban, True)
    sent_messages = self._published(fake)

    self.assertEqual(len(sent_messages), 1)
    _, msg = sent_messages[0]
    self.assertEqual(msg.liveMapDataSP.roadType, RoadType.urban)

  def test_publish_road_type_defaults_to_unknown(self):
    fake = _fake_map_data("", RoadType.unknown, False)
    sent_messages = self._published(fake)

    _, msg = sent_messages[0]
    self.assertEqual(msg.liveMapDataSP.roadType, RoadType.unknown)
