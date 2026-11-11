"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import re

from openpilot.cereal import custom

RoadType = custom.LiveMapDataSP.RoadType

# Speed thresholds are derived from their mph intent so the comparison is inclusive at
# the stated posted limit (e.g. a true 55 mph road must satisfy speed >= threshold).
MPH = 0.44704  # m/s per mph

HIGHWAY_SPEED_THRESHOLD = 55 * MPH  # 24.5872 m/s — unnamed roads at >= 55 mph are highway
HIGHWAY_NAME_SPEED_THRESHOLD = 50 * MPH  # 22.352 m/s — named routes at >= 50 mph are highway
URBAN_SPEED_CEILING = 45 * MPH  # 20.1168 m/s — 0 < limit <= 45 mph is urban

# Interstate / freeway patterns — always classified as interstate regardless of speed
_INTERSTATE_RE = re.compile(
  r'\bI-\d+\b|\bInterstate\s+\d+|\bFreeway\b|\bExpressway\b',
  re.IGNORECASE,
)

# US highway and state route patterns — classified as highway only above speed threshold.
# Includes two-letter state prefixes: CA-1, TX-130, NY-17
_HIGHWAY_NAME_RE = re.compile(
  r'\bUS[-\s]?\d+|\bUS\s+(Route|Highway)\s+\d+|\bU\.S\.\s+\d+|\bSR[-\s]?\d+|\bState\s+Route\s+\d+|\bState\s+Highway\s+\d+|\b[A-Z]{2}-\d+\b',
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

  if 0 < speed_limit <= URBAN_SPEED_CEILING:
    return RoadType.urban

  # Ambiguous zone (45-55 mph unnamed) or no data at all
  if speed_limit > 0:
    return RoadType.unknown

  # If we have a road name but no speed and no pattern match,
  # it's likely a local road
  if name:
    return RoadType.urban

  return RoadType.unknown
