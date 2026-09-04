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

"""Unit tests for the ROS-free traffic-light position matcher."""

import textwrap

from autoware_carla_interface.modules.traffic_light_matcher import MapTrafficLights
from autoware_carla_interface.modules.traffic_light_matcher import MatchResult
from autoware_carla_interface.modules.traffic_light_matcher import load_map_traffic_lights
from autoware_carla_interface.modules.traffic_light_matcher import match_traffic_lights


def _map(heads):
    """Build a MapTrafficLights from {way_id: ((x, y), {group_id, ...})}."""
    m = MapTrafficLights()
    for way_id, (pos, groups) in heads.items():
        m.head_positions[way_id] = pos
        m.head_groups[way_id] = set(groups)
    return m


def _status(result, actor_id):
    return next(e["status"] for e in result.entries if e["actor_id"] == actor_id)


def test_exact_hit_resolves_to_all_groups_of_the_shared_head():
    # One physical head shared by two regulatory elements (two approaches).
    m = _map({"w1": ((10.0, 10.0), {100, 101})})
    result = match_traffic_lights([("a", 5, (10.0, 10.0))], m)
    assert result.assignments["a"] == [100, 101]
    assert _status(result, "a") == MatchResult.MATCHED


def test_isolated_head_matches_with_noise():
    m = _map({"w1": ((0.0, 0.0), {1}), "w2": ((100.0, 0.0), {2})})
    result = match_traffic_lights([("a", None, (0.7, -0.5))], m)
    assert result.assignments["a"] == [1]


def test_midpoint_between_disjoint_signals_is_ambiguous():
    # Two heads 2 m apart governing different regulatory elements; a light halfway
    # between them cannot be attributed and must be dropped, not guessed.
    m = _map({"w1": ((0.0, 0.0), {1}), "w2": ((2.0, 0.0), {2})})
    result = match_traffic_lights([("a", 9, (1.0, 0.0))], m)
    assert "a" not in result.assignments
    assert _status(result, "a") == MatchResult.AMBIGUOUS


def test_close_neighbor_sharing_groups_is_not_ambiguous():
    # A neighbouring head of the *same* regulatory elements must not trigger the
    # ambiguity guard: resolving to either head gives the same group ids.
    m = _map({"w1": ((0.0, 0.0), {1, 2}), "w2": ((1.0, 0.0), {1, 2})})
    result = match_traffic_lights([("a", None, (0.1, 0.0))], m)
    assert result.assignments["a"] == [1, 2]


def test_clear_winner_over_nearby_disjoint_head_still_matches():
    # Light sits essentially on its own head (0.1 m); a different signal 1.5 m away
    # should not defeat the match thanks to the ratio-based ambiguity test.
    m = _map({"w1": ((0.0, 0.0), {1}), "w2": ((1.5, 0.0), {2})})
    result = match_traffic_lights([("a", None, (0.1, 0.0))], m)
    assert result.assignments["a"] == [1]


def test_too_far_is_unmatched():
    m = _map({"w1": ((0.0, 0.0), {1})})
    result = match_traffic_lights([("a", None, (50.0, 50.0))], m, distance_threshold=5.0)
    assert "a" not in result.assignments
    assert _status(result, "a") == MatchResult.TOO_FAR


def test_empty_map_reports_no_head():
    result = match_traffic_lights([("a", None, (0.0, 0.0))], MapTrafficLights())
    assert _status(result, "a") == MatchResult.NO_HEAD


_OSM = textwrap.dedent("""\
    <?xml version="1.0" encoding="UTF-8"?>
    <osm version="0.6">
      <node id="1"><tag k="local_x" v="0.0"/><tag k="local_y" v="0.0"/></node>
      <node id="2"><tag k="local_x" v="2.0"/><tag k="local_y" v="0.0"/></node>
      <node id="3"><tag k="local_x" v="10.0"/><tag k="local_y" v="0.0"/></node>
      <node id="4"><tag k="local_x" v="12.0"/><tag k="local_y" v="0.0"/></node>
      <way id="100"><nd ref="1"/><nd ref="2"/><tag k="type" v="traffic_light"/></way>
      <way id="200"><nd ref="3"/><nd ref="4"/><tag k="type" v="traffic_light"/></way>
      <relation id="500">
        <member type="way" role="refers" ref="100"/>
        <tag k="type" v="regulatory_element"/>
        <tag k="subtype" v="traffic_light"/>
      </relation>
      <relation id="501">
        <member type="way" role="refers" ref="100"/>
        <member type="way" role="refers" ref="200"/>
        <tag k="type" v="regulatory_element"/>
        <tag k="subtype" v="traffic_light"/>
      </relation>
      <relation id="999">
        <member type="way" role="refers" ref="200"/>
        <tag k="type" v="regulatory_element"/>
        <tag k="subtype" v="crosswalk"/>
      </relation>
    </osm>
    """)


def test_load_map_parses_heads_and_shared_groups(tmp_path):
    osm = tmp_path / "map.osm"
    osm.write_text(_OSM)
    m = load_map_traffic_lights(str(osm))

    # Two traffic-light heads (ways 100 and 200); the crosswalk relation is ignored.
    assert len(m) == 2
    # Head 100 is referenced by both regulatory elements 500 and 501.
    assert m.head_groups["100"] == {500, 501}
    # Head 200 is referenced by the traffic-light element 501 only (not the crosswalk).
    assert m.head_groups["200"] == {501}
    # Centroid is the mean of the way's node local coordinates.
    assert m.head_positions["100"] == (1.0, 0.0)
    assert m.group_count == 2


def test_id_map_override_semantics_via_matcher(tmp_path):
    osm = tmp_path / "map.osm"
    osm.write_text(_OSM)
    m = load_map_traffic_lights(str(osm))
    # A CARLA light on head 100 resolves to both 500 and 501.
    result = match_traffic_lights([("a", 7, (1.0, 0.0))], m)
    assert result.assignments["a"] == [500, 501]
