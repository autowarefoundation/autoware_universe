# Kashiwanoha map

An Autoware vector map for the Kashiwanoha Campus area, shipped so that the CARLA interface can be
exercised on a map that started life as lanelet2, rather than only on the CARLA towns whose lanelet2
maps are derived from the simulator's own OpenDRIVE.

There is no point cloud map here. This map is meant for the ground-truth localization setup, where
`carla_state_publisher` provides the pose and no scan matching runs, so a point cloud is never read.

Because `pointcloud_map_loader` is launched unconditionally, it will log

```text
[ERROR] [map.pointcloud_map_loader]: PCD load failed: <map_path>/pointcloud_map.pcd
```

and that node will not come up. Nothing downstream of it is needed here: `/autoware/map` stays OK in
the diagnostic graph, routing and engage succeed, and the only subscriber to the point cloud map
topic is RViz, which simply shows no point cloud. Drop a point cloud map into this directory if you
want one for display or for a scan-matching setup.

## Files

| File                      | Origin                                                              |
| ------------------------- | ------------------------------------------------------------------- |
| `lanelet2_map.osm`        | Redistributed unmodified, see [Attribution](#attribution)           |
| `map_projector_info.yaml` | Written for this package, see [Coordinate frame](#coordinate-frame) |
| `kashiwanoha.xodr`        | Converted from `lanelet2_map.osm`, see [CARLA map](#carla-map)      |

## Attribution

`lanelet2_map.osm` is copied verbatim from [tier4/scenario_simulator_v2], which is licensed under the
Apache License 2.0, the same license as this repository.

|            |                                                                             |
| ---------- | --------------------------------------------------------------------------- |
| Repository | <https://github.com/tier4/scenario_simulator_v2>                            |
| Path       | `map/kashiwanoha_map/map/private_road_and_walkway_ele_fix/lanelet2_map.osm` |
| Commit     | `6a805e470f46bc48b248a3aaa745e4a54562e7ac`                                  |
| SHA-256    | `b7f42a10e4fb4478c1c6749ac642cef946f5cfd77841202124f951289f978243`          |

The file is **not modified**. The SHA-256 above is of the file as shipped here, and it matches the
upstream blob at that commit.

`scenario_simulator_v2` ships several variants of this map. This is the
`private_road_and_walkway_ele_fix` variant, which adds five lanelets covering one private road and
one walkway and corrects their elevation. It disagrees with the plain `map/lanelet2_map.osm` variant
by up to 4.9 m over 20 lanelets, so do not mix the two: anything derived from this map, including an
OpenDRIVE conversion, has to come from this same variant.

[tier4/scenario_simulator_v2]: https://github.com/tier4/scenario_simulator_v2

## Coordinate frame

`map_projector_info.yaml` places the map origin at the centre of the mapped area rather than at the
MGRS grid origin:

|            |                                                                           |
| ---------- | ------------------------------------------------------------------------- |
| Projector  | `LocalCartesianUTM`                                                       |
| Origin     | lat `35.9033135426554`, lon `139.93338978245356`                          |
| Equivalent | MGRS grid `54SVE`, easting 3750, northing 73750 (UTM 54N 403750, 3973750) |

The reason for the local origin is CARLA: Unreal Engine stores positions as 32-bit floats in
centimetres, so a map placed 74 km from the origin loses precision. Keeping the origin at the centre
of the map keeps every coordinate within roughly ±100 m.

The CARLA map has to be built against this same origin. If you convert this lanelet2 map to
OpenDRIVE yourself, give the converter the origin above.

## CARLA map

Autoware needs the lanelet2 map above; CARLA needs the same road network in its own form. Two ways
to give it one.

### OpenDRIVE, from the bundled `kashiwanoha.xodr`

CARLA can build a world straight from OpenDRIVE, so this needs nothing beyond what is in this
directory. Load it before starting the bridge, then launch the bridge against the world it created:

```python
import carla

client = carla.Client("localhost", 2000)
client.set_timeout(300.0)
with open("kashiwanoha.xodr") as f:
    client.generate_opendrive_world(
        f.read(),
        carla.OpendriveGenerationParameters(
            vertex_distance=2.0,
            max_road_length=500.0,
            wall_height=0.0,
            additional_width=0.6,
            smooth_junctions=True,
            enable_mesh_visibility=True,
            enable_pedestrian_navigation=False,
        ),
    )
```

**Keep `wall_height` at 0.** CARLA raises boundary walls per road, and this network is one lane per
road, so any positive height puts a wall between adjacent lanes. A vehicle that touches one is
thrown into the air.

The world CARLA generates is named `OpenDriveMap`, so pass `carla_map:=OpenDriveMap` to the bridge
launch to stop it from loading a different map over the one you just built.

The generated world is geometry only: roads, sidewalks and markings, no buildings or vegetation.

### A CARLA content package

A prebuilt Unreal package of the same area, with the same origin, gives a better looking world and
loads faster. One is being prepared for release; this section will name it once it is published.
With it installed, no OpenDRIVE step is needed, and the bridge loads it like any built-in town
through `carla_map`.

### Provenance of `kashiwanoha.xodr`

Converted from the `lanelet2_map.osm` in this directory with
[tier4/autoware_lanelet2_to_opendrive] v2.62.0, targeting CARLA, with the origin from
[Coordinate frame](#coordinate-frame) and left-hand traffic. No manual edits were made to the
network before or after the conversion: every merge, move, delete and remove operation the converter
offers was left empty. The result passes the ASAM OpenDRIVE quality checker.

[tier4/autoware_lanelet2_to_opendrive]: https://github.com/tier4/autoware_lanelet2_to_opendrive

## Extent and limitations

The map covers 143 m north to south by 163 m east to west and holds 86 lanelets: 81 road, 4
crosswalk and 1 walkway. It carries one traffic light regulatory element, made of three light units,
and one traffic sign regulatory element.

That suits short functional checks. It is too small for long-distance driving, and it has too few
signals for signal-heavy scenarios. Being a survey of a real site, its lane widths and curvature
vary, unlike the CARLA towns.
