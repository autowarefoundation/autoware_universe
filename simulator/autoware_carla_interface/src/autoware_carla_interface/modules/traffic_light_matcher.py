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

from collections import namedtuple
import math
import xml.etree.ElementTree as ET

# Immutable per-run matching context, so the per-head classifier takes one context
# argument instead of a long parameter list. ``head_items`` is the map heads
# pre-materialised as [(way_id, (x, y)), ...] so it is built once, not per head.
_MatchContext = namedtuple(
    "_MatchContext", ["map_lights", "head_items", "distance_threshold", "ambiguity_ratio"]
)


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


def _parse_local_nodes(root):
    """Node id -> (local_x, local_y) for nodes that carry both tags."""
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
    return nodes


def _parse_ways(root):
    """Way id -> list of node ids."""
    return {
        way.get("id"): [nd.get("ref") for nd in way.findall("nd")] for way in root.findall("way")
    }


def _is_traffic_light_relation(relation):
    tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
    return tags.get("type") == "regulatory_element" and tags.get("subtype") == "traffic_light"


def _refers_way_ids(relation):
    """Way ids referenced with role ``refers`` (the physical light heads)."""
    return [m.get("ref") for m in relation.findall("member") if m.get("role") == "refers"]


def _way_centroid(way_id, ways, nodes):
    """Mean (x, y) of a way's known nodes, or None if it has none."""
    points = [nodes[ref] for ref in ways.get(way_id, []) if ref in nodes]
    if not points:
        return None
    return (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))


def _add_relation_heads(relation, ways, nodes, result):
    """Record every ``refers`` head of one traffic-light relation into ``result``."""
    group_id = int(relation.get("id"))
    for way_id in _refers_way_ids(relation):
        centroid = _way_centroid(way_id, ways, nodes)
        if centroid is None:
            continue
        result.head_positions.setdefault(way_id, centroid)
        result.head_groups.setdefault(way_id, set()).add(group_id)


def load_map_traffic_lights(osm_path):
    """Parse a lanelet2 map and return its traffic-light heads.

    The map is read as raw OSM XML rather than through the lanelet2 library on purpose:
    it avoids pulling in the C++ regulatory-element registration (which is easy to get
    out of sync across ROS/Python versions) and reads ``local_x``/``local_y`` directly,
    sidestepping any projector mismatch between this process and the map loader.
    """
    root = ET.parse(osm_path).getroot()
    nodes = _parse_local_nodes(root)
    ways = _parse_ways(root)

    result = MapTrafficLights()
    for relation in root.findall("relation"):
        if _is_traffic_light_relation(relation):
            _add_relation_heads(relation, ways, nodes, result)

    return result


def _parse_id(text, what):
    """Parse one integer id, raising ``ValueError`` naming ``what`` was expected."""
    try:
        return int(text.strip())
    except ValueError:
        raise ValueError(f"'{text.strip()}' is not an integer {what}")


def _parse_id_map_groups(groups_str):
    """Parse the ``group_id[|group_id...]`` half of an id-map entry into a set."""
    groups = {
        _parse_id(token, "traffic-light group id")
        for token in groups_str.split("|")
        if token.strip()
    }
    if not groups:
        raise ValueError("no group id after ':'")
    return groups


def _parse_id_map_entry(item):
    """Parse one ``opendrive_id:group_id[|group_id...]`` entry.

    Returns ``(opendrive_id, {group_id, ...})``, or raises ``ValueError`` describing
    what is wrong with the entry so the caller can report it and skip it.
    """
    opendrive_str, separator, groups_str = item.partition(":")
    if not separator:
        raise ValueError("missing ':', expected 'opendrive_id:group_id[|group_id...]'")
    return _parse_id(opendrive_str, "OpenDRIVE signal id"), _parse_id_map_groups(groups_str)


def parse_id_map_override(raw, on_invalid=None):
    """Parse a ``traffic_light.id_map`` string into ``{opendrive_id: [group_id, ...]}``.

    Format is ``opendrive_id:group_id[|group_id...],...``. A single OpenDRIVE signal id
    may list several group ids (separated by ``|``) so a physical light shared by
    multiple regulatory elements can be pinned to all of them, matching the position
    matcher's shared-head behaviour. Repeated keys are merged.

    A malformed entry is skipped rather than raising: a typo in this parameter must not
    take the bridge down on the first simulation tick, and an entry such as ``12:`` must
    not silently override a light with an empty group list. Every rejected entry is
    reported through the optional ``on_invalid`` callback (e.g. a logger's ``warning``),
    which receives one human-readable message per bad entry. Entries that parse fine are
    kept, so one typo does not discard the rest of the table.
    """
    override = {}
    for item in str(raw or "").split(","):
        item = item.strip()
        if not item:
            continue
        try:
            opendrive_id, groups = _parse_id_map_entry(item)
        except ValueError as error:
            if on_invalid is not None:
                on_invalid(f"ignoring malformed id_map entry '{item}': {error}")
            continue
        override.setdefault(opendrive_id, set()).update(groups)
    return {opendrive_id: sorted(groups) for opendrive_id, groups in override.items()}


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


def _is_ambiguous(nearest_dist, second_dist, ambiguity_ratio):
    """Whether the runner-up head is close enough that the winner cannot be trusted.

    ``second_dist`` is the distance to the closest head resolving to a different answer,
    or None when there is none. A tie has no winner -- which head ranks first would come
    down to the order the ways appear in the .osm -- so equal distances are ambiguous
    whatever the ratio, including two heads sitting on the same point where the ratio
    test degenerates to ``0 > 0``.
    """
    if second_dist is None:
        return False
    if nearest_dist >= second_dist:
        return True
    return nearest_dist > ambiguity_ratio * second_dist


def _classify_head(head, ctx):
    """Decide how one CARLA head resolves, returning (entry, group_ids or None).

    ``head`` is an ``(actor_id, opendrive_id, (x, y))`` tuple and ``ctx`` is a
    :data:`_MatchContext`. See :func:`match_traffic_lights` for the threshold / ratio
    semantics. Kept as a small helper so the matching loop stays flat and each outcome
    is a single branch.
    """
    actor_id, opendrive_id, point = head
    entry = {"status": None, "actor_id": actor_id, "opendrive_id": opendrive_id}

    ranked = sorted(
        ((_distance(point, pos), way_id) for way_id, pos in ctx.head_items),
        key=lambda item: item[0],
    )
    if not ranked:
        entry["status"] = MatchResult.NO_HEAD
        return entry, None

    nearest_dist, nearest_way = ranked[0]
    entry["nearest"] = nearest_dist
    if nearest_dist > ctx.distance_threshold:
        entry["status"] = MatchResult.TOO_FAR
        return entry, None

    nearest_groups = ctx.map_lights.head_groups[nearest_way]
    # Closest head that would resolve to a *different answer* than the winner. A head
    # is a genuine alternative unless its group set is exactly equal to the winner's:
    # only then does matching it publish the same group ids, so a neighbouring head of
    # the same approach is ignored, while an overlapping-but-unequal or disjoint set
    # (e.g. {500, 501} vs {501}, or the light across the intersection) still makes the
    # match ambiguous.
    second_dist = next(
        (d for d, way_id in ranked[1:] if ctx.map_lights.head_groups[way_id] != nearest_groups),
        None,
    )
    entry["second"] = second_dist
    if _is_ambiguous(nearest_dist, second_dist, ctx.ambiguity_ratio):
        entry["status"] = MatchResult.AMBIGUOUS
        return entry, None

    group_ids = sorted(nearest_groups)
    entry["status"] = MatchResult.MATCHED
    entry["group_ids"] = group_ids
    return entry, group_ids


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
        ``nearest_dist > ambiguity_ratio * second_dist`` (an exact tie is always
        ambiguous, whatever the ratio). Using a ratio rather than an
        absolute margin keeps confident matches (a CARLA light sitting essentially on
        its own head, so ``nearest_dist`` is tiny) even when another signal is only a
        metre or two away, while still rejecting a light that falls roughly midway
        between two genuinely different signals. Lower is stricter.

    Returns
    -------
    MatchResult
    """
    ctx = _MatchContext(
        map_lights=map_lights,
        head_items=list(map_lights.head_positions.items()),  # [(way_id, (x, y)), ...]
        distance_threshold=distance_threshold,
        ambiguity_ratio=ambiguity_ratio,
    )
    result = MatchResult()
    for head in carla_heads:
        entry, group_ids = _classify_head(head, ctx)
        result.entries.append(entry)
        if group_ids is not None:
            result.assignments[head[0]] = group_ids

    return result
