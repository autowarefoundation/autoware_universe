# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Position-based matching of CARLA traffic lights to lanelet2 regulatory elements.

The CARLA server exposes each traffic light as an actor with a physical light-head
pose, while Autoware keys traffic-light states by ``traffic_light_group_id`` (the id
of a ``traffic_light`` regulatory element in the lanelet2 map). Rather than assume the
CARLA OpenDRIVE signal id equals the regulatory element id (only true for maps
auto-generated straight from the same OpenDRIVE) or require a hand-written id table,
this module discovers the mapping geometrically: it matches each CARLA light head to
the nearest lanelet2 light head and copies its state onto every regulatory element that
references that head.

The matcher is deliberately conservative. A CARLA head is bound to a lanelet2 head only
when a single candidate is clearly closest; if the second-closest head from a *different*
physical light is nearly as close (the classic "signal across the intersection" case) or
nothing is within the distance threshold, the head is left unmatched and reported, never
guessed. Unmatched heads can then be pinned individually via the id-map override instead
of hand-authoring the whole table.

Coordinates
-----------
lanelet2 node ``local_x``/``local_y`` tags are the Autoware map-frame coordinates (for an
MGRS map they are exactly the MGRS easting/northing the map loader produces, verified
against ``map_projector_info.yaml``). The caller is responsible for expressing the CARLA
head positions in the same map frame (see ``carla_location_to_ros_point``) before calling
``match``; the matcher itself is coordinate-frame agnostic and only compares 2-D points.
"""

import math
import xml.etree.ElementTree as ET


class MapTrafficLights:
    """Traffic-light heads parsed from a lanelet2 (.osm) map.

    A "head" is one physical light bar, i.e. one ``way`` referenced with role ``refers``
    by a ``traffic_light`` regulatory element. The same head is frequently shared by
    several regulatory elements (one per approaching lane / stop line), so each head
    carries the *set* of group ids that reference it; matching a CARLA light to a head
    therefore resolves to one or more Autoware group ids at once.
    """

    def __init__(self):
        # refers-way id (str) -> (x, y) head centroid in map frame
        self.head_positions = {}
        # refers-way id (str) -> set of regulatory-element (group) ids referencing it
        self.head_groups = {}

    def __len__(self):
        return len(self.head_positions)

    @property
    def group_count(self):
        return len({gid for gids in self.head_groups.values() for gid in gids})


def load_map_traffic_lights(osm_path):
    """Parse a lanelet2 map and return its traffic-light heads.

    The map is read as raw OSM XML rather than through the lanelet2 library on purpose:
    it avoids pulling in the C++ regulatory-element registration (which is easy to get
    out of sync across ROS/Python versions) and reads ``local_x``/``local_y`` directly,
    sidestepping any projector mismatch between this process and the map loader.
    """
    root = ET.parse(osm_path).getroot()

    # node id -> (local_x, local_y)
    nodes = {}
    for node in root.findall("node"):
        local_x = local_y = None
        for tag in node.findall("tag"):
            key = tag.get("k")
            if key == "local_x":
                local_x = float(tag.get("v"))
            elif key == "local_y":
                local_y = float(tag.get("v"))
        if local_x is not None and local_y is not None:
            nodes[node.get("id")] = (local_x, local_y)

    # way id -> list of node ids
    ways = {
        way.get("id"): [nd.get("ref") for nd in way.findall("nd")] for way in root.findall("way")
    }

    result = MapTrafficLights()
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "traffic_light":
            continue
        group_id = int(relation.get("id"))
        for member in relation.findall("member"):
            if member.get("role") != "refers":
                continue
            way_id = member.get("ref")
            points = [nodes[ref] for ref in ways.get(way_id, []) if ref in nodes]
            if not points:
                continue
            if way_id not in result.head_positions:
                cx = sum(p[0] for p in points) / len(points)
                cy = sum(p[1] for p in points) / len(points)
                result.head_positions[way_id] = (cx, cy)
                result.head_groups[way_id] = set()
            result.head_groups[way_id].add(group_id)

    return result


class MatchResult:
    """Outcome of matching CARLA light heads against the map, for one run.

    ``assignments`` maps a CARLA actor id to the sorted list of Autoware group ids it
    resolved to. ``entries`` records the per-actor decision (matched / ambiguous /
    too_far / no_head) so the caller can log a human-readable report.
    """

    MATCHED = "matched"
    AMBIGUOUS = "ambiguous"
    TOO_FAR = "too_far"
    NO_HEAD = "no_head"

    def __init__(self):
        self.assignments = {}  # actor_id -> [group_id, ...]
        self.entries = (
            []
        )  # list of dict(status, actor_id, opendrive_id, group_ids, nearest, second)

    @property
    def matched_actor_count(self):
        return len(self.assignments)


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def match_traffic_lights(
    carla_heads,
    map_lights,
    distance_threshold=5.0,
    ambiguity_ratio=0.6,
):
    """Match CARLA light heads to lanelet2 heads by position.

    Parameters
    ----------
    carla_heads : iterable of (actor_id, opendrive_id, (x, y))
        CARLA traffic-light heads already expressed in the map frame. ``opendrive_id``
        may be ``None`` and is only carried through for reporting / override lookup.
    map_lights : MapTrafficLights
        Parsed lanelet2 traffic-light heads.
    distance_threshold : float
        Maximum head-to-head distance (metres) accepted as a match.
    ambiguity_ratio : float
        A match is ambiguous when the closest head that resolves to a *different set of
        regulatory elements* is nearly as close as the winner, i.e. when
        ``nearest_dist > ambiguity_ratio * second_dist``. Using a ratio rather than an
        absolute margin keeps confident matches (a CARLA light sitting essentially on
        its own head, so ``nearest_dist`` is tiny) even when another signal is only a
        metre or two away, while still rejecting a light that falls roughly midway
        between two genuinely different signals. Lower is stricter.

    Returns
    -------
    MatchResult
    """
    result = MatchResult()
    head_items = list(map_lights.head_positions.items())  # [(way_id, (x, y)), ...]

    for actor_id, opendrive_id, point in carla_heads:
        # Rank every map head by distance to this CARLA head.
        ranked = sorted(
            ((_distance(point, pos), way_id) for way_id, pos in head_items),
            key=lambda item: item[0],
        )
        if not ranked:
            result.entries.append(
                {"status": MatchResult.NO_HEAD, "actor_id": actor_id, "opendrive_id": opendrive_id}
            )
            continue

        nearest_dist, nearest_way = ranked[0]
        nearest_groups = map_lights.head_groups[nearest_way]
        # Closest head that would resolve to a *different* set of group ids, i.e. a
        # genuinely different signal (the light across the intersection), not just the
        # neighbouring head of the same approach, which shares the same regulatory
        # elements and would give the same answer. Only such a head makes a match
        # ambiguous.
        second_dist = next(
            (
                d
                for d, way_id in ranked[1:]
                if map_lights.head_groups[way_id].isdisjoint(nearest_groups)
            ),
            None,
        )

        if nearest_dist > distance_threshold:
            result.entries.append(
                {
                    "status": MatchResult.TOO_FAR,
                    "actor_id": actor_id,
                    "opendrive_id": opendrive_id,
                    "nearest": nearest_dist,
                }
            )
            continue

        if second_dist is not None and nearest_dist > ambiguity_ratio * second_dist:
            result.entries.append(
                {
                    "status": MatchResult.AMBIGUOUS,
                    "actor_id": actor_id,
                    "opendrive_id": opendrive_id,
                    "nearest": nearest_dist,
                    "second": second_dist,
                }
            )
            continue

        group_ids = sorted(map_lights.head_groups[nearest_way])
        result.assignments[actor_id] = group_ids
        result.entries.append(
            {
                "status": MatchResult.MATCHED,
                "actor_id": actor_id,
                "opendrive_id": opendrive_id,
                "group_ids": group_ids,
                "nearest": nearest_dist,
                "second": second_dist,
            }
        )

    return result
