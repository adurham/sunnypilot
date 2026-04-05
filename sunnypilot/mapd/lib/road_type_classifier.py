"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import re

from cereal import custom

RoadType = custom.LiveMapDataSP.RoadType

# Speed thresholds in m/s
HIGHWAY_SPEED_THRESHOLD = 24.6  # ~55 mph
HIGHWAY_NAME_SPEED_THRESHOLD = 22.3  # ~50 mph
URBAN_SPEED_CEILING = 20.1  # ~45 mph

# Interstate / freeway patterns — always classified as interstate regardless of speed
_INTERSTATE_RE = re.compile(
  r'\bI-\d+\b'
  r'|\bInterstate\s+\d+'
  r'|\bFreeway\b'
  r'|\bExpressway\b',
  re.IGNORECASE,
)

# US highway and state route patterns — classified as highway only above speed threshold
_HIGHWAY_NAME_RE = re.compile(
  r'\bUS[-\s]?\d+'
  r'|\bUS\s+(Route|Highway)\s+\d+'
  r'|\bU\.S\.\s+\d+'
  r'|\bSR[-\s]?\d+'
  r'|\bState\s+Route\s+\d+'
  r'|\bState\s+Highway\s+\d+'
  r'|\b[A-Z]{2}-\d+\b',  # Two-letter state prefix: CA-1, TX-130, NY-17
  re.IGNORECASE,
)


def classify_road_type(road_name: str, speed_limit: float) -> str:
  """Classify road type from road name and speed limit.

  Args:
    road_name: OSM road name from mapd (e.g., "Interstate 5", "Main Street")
    speed_limit: Current speed limit in m/s (0 if unavailable)

  Returns:
    RoadType enum value (capnp string enum)
  """
  name = road_name.strip()

  # Interstate patterns always win
  if name and _INTERSTATE_RE.search(name):
    return RoadType.interstate

  # Highway name patterns — require minimum speed to distinguish from
  # state routes passing through towns at low speed
  if name and _HIGHWAY_NAME_RE.search(name):
    if speed_limit >= HIGHWAY_NAME_SPEED_THRESHOLD:
      return RoadType.highway
    if speed_limit > 0:
      return RoadType.urban
    # Named highway route but no speed data — call it highway
    return RoadType.highway

  # Speed-only fallback for unnamed or unrecognized roads
  if speed_limit >= HIGHWAY_SPEED_THRESHOLD:
    return RoadType.highway

  if speed_limit > 0 and speed_limit < URBAN_SPEED_CEILING:
    return RoadType.urban

  # Ambiguous zone (45-55 mph unnamed) or no data at all
  if speed_limit > 0:
    return RoadType.unknown

  # If we have a road name but no speed and no pattern match,
  # it's likely a local road
  if name:
    return RoadType.urban

  return RoadType.unknown
