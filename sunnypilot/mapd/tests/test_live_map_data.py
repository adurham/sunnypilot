"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest
from unittest.mock import MagicMock, patch

from cereal import custom

RoadType = custom.LiveMapDataSP.RoadType

MPH_TO_MS = 0.44704


class TestOsmMapDataRoadType:
  """Verify OsmMapData.get_current_road_type() wires params through the classifier."""

  @pytest.fixture
  def osm_map_data(self):
    with patch('openpilot.sunnypilot.mapd.live_map_data.osm_map_data.BaseMapData.__init__'):
      from openpilot.sunnypilot.mapd.live_map_data.osm_map_data import OsmMapData
      obj = OsmMapData.__new__(OsmMapData)
      obj.mem_params = MagicMock()
      return obj

  def test_interstate_from_params(self, osm_map_data):
    osm_map_data.mem_params.get.side_effect = lambda key: {
      "RoadName": "Interstate 5",
      "MapSpeedLimit": "29.1",
    }.get(key)
    assert osm_map_data.get_current_road_type() == RoadType.interstate

  def test_highway_from_params(self, osm_map_data):
    osm_map_data.mem_params.get.side_effect = lambda key: {
      "RoadName": "US-101",
      "MapSpeedLimit": str(55 * MPH_TO_MS),
    }.get(key)
    assert osm_map_data.get_current_road_type() == RoadType.highway

  def test_urban_from_params(self, osm_map_data):
    osm_map_data.mem_params.get.side_effect = lambda key: {
      "RoadName": "Main Street",
      "MapSpeedLimit": str(35 * MPH_TO_MS),
    }.get(key)
    assert osm_map_data.get_current_road_type() == RoadType.urban

  def test_unknown_when_no_data(self, osm_map_data):
    osm_map_data.mem_params.get.side_effect = lambda key: {
      "RoadName": None,
      "MapSpeedLimit": None,
    }.get(key)
    assert osm_map_data.get_current_road_type() == RoadType.unknown


class TestBaseMapDataPublish:
  """Verify publish() sets roadType on the cereal message."""

  def test_publish_includes_road_type(self):
    from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData

    # Create a concrete subclass for testing
    class FakeMapData(BaseMapData):
      def __init__(self):
        # Skip real init — mock messaging
        self.sm = MagicMock()
        self.pm = MagicMock()

      def update_location(self):
        pass

      def get_current_speed_limit(self):
        return 29.1  # ~65 mph

      def get_next_speed_limit_and_distance(self):
        return 0.0, 0.0

      def get_current_road_name(self):
        return "Interstate 5"

      def get_current_road_type(self):
        return RoadType.interstate

    fake = FakeMapData()
    fake.sm['liveLocationKalman'].gpsOK = True

    # Capture the message sent to pm.send
    sent_messages = []
    fake.pm.send.side_effect = lambda topic, msg: sent_messages.append((topic, msg))

    fake.publish()

    assert len(sent_messages) == 1
    topic, msg = sent_messages[0]
    assert topic == 'liveMapDataSP'
    assert msg.liveMapDataSP.roadType == RoadType.interstate
    assert msg.liveMapDataSP.roadName == "Interstate 5"

  def test_publish_road_type_urban(self):
    from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData

    class FakeMapData(BaseMapData):
      def __init__(self):
        self.sm = MagicMock()
        self.pm = MagicMock()

      def update_location(self):
        pass

      def get_current_speed_limit(self):
        return 15.6  # ~35 mph

      def get_next_speed_limit_and_distance(self):
        return 0.0, 0.0

      def get_current_road_name(self):
        return "Elm Street"

      def get_current_road_type(self):
        return RoadType.urban

    fake = FakeMapData()
    fake.sm['liveLocationKalman'].gpsOK = True

    sent_messages = []
    fake.pm.send.side_effect = lambda topic, msg: sent_messages.append((topic, msg))

    fake.publish()

    assert len(sent_messages) == 1
    _, msg = sent_messages[0]
    assert msg.liveMapDataSP.roadType == RoadType.urban

  def test_publish_road_type_defaults_to_unknown(self):
    from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData

    class FakeMapData(BaseMapData):
      def __init__(self):
        self.sm = MagicMock()
        self.pm = MagicMock()

      def update_location(self):
        pass

      def get_current_speed_limit(self):
        return 0.0

      def get_next_speed_limit_and_distance(self):
        return 0.0, 0.0

      def get_current_road_name(self):
        return ""

      def get_current_road_type(self):
        return RoadType.unknown

    fake = FakeMapData()
    fake.sm['liveLocationKalman'].gpsOK = False

    sent_messages = []
    fake.pm.send.side_effect = lambda topic, msg: sent_messages.append((topic, msg))

    fake.publish()

    _, msg = sent_messages[0]
    assert msg.liveMapDataSP.roadType == RoadType.unknown
