# Kashiwanoha test map

`kashiwanoha.xodr` is the OpenDRIVE for a real site, the Kashiwanoha Campus area in Kashiwa, Chiba,
converted from a lanelet2 map this repository already carries. CARLA has no map of this area, so it
needs the road network in a form it can build a world from.

Follow the [package README](../../README.md) for everything else: installing CARLA, building, and
launching. Only what differs from the CARLA towns described there is written below.

## Preparing the map directory

The same directory the package README sets up for Town01, with these contents instead:

| File                      | Where it comes from                                                                        |
| ------------------------- | ------------------------------------------------------------------------------------------ |
| `lanelet2_map.osm`        | Copy of `planning/autoware_diffusion_planner/test_map/lanelet2_map.osm` in this repository |
| `map_projector_info.yaml` | Written by hand, with the origin below                                                     |

Town01 uses `projector_type: Local`. This map is in latitude and longitude, so it needs its origin
stated:

```yaml
projector_type: LocalCartesianUTM
vertical_datum: WGS84
map_origin:
  latitude: 35.9033135426554
  longitude: 139.93338978245356
  altitude: 0.0
```

This is the origin `kashiwanoha.xodr` was converted against, which is what lets Autoware and CARLA
agree on where things are. Changing it on one side only puts them in different places.

There is no point cloud map here, and a ground truth localization setup does not need one:
`pointcloud_map_loader` logs `PCD load failed` and does not come up, and routing, engage and driving
work without it.

That holds as long as nothing downstream reads the point cloud map. A perception stack that does
will stall instead: its compare-map filters wait on a service only `pointcloud_map_loader` offers,
so no occupancy grid is published and the behavior path planner stops at
`waiting for occupancy_grid_map`. Add a point cloud map to the directory if you need that path.

## Building the CARLA world

CARLA has no map of this area to load by name, so the world is built from the OpenDRIVE. Do this
before launching:

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

**Keep `wall_height` at 0.** CARLA raises boundary walls per road, and every one of this network's
190 roads holds a single lane, so any positive height puts a wall between adjacent lanes. A vehicle
that touches one is thrown into the air.

The world this generates is named `OpenDriveMap`, so add `carla_map:=OpenDriveMap` to the launch
command in the package README, alongside a `map_path` pointing at the directory above. The world is
geometry only: roads, sidewalks and markings, no buildings or vegetation.

## Provenance

Converted from `planning/autoware_diffusion_planner/test_map/lanelet2_map.osm` (SHA-256
`4fe358f2d1e182dc76de296e5619086f8d8ad2fcab3e6d8f99f6e3d724a84e6d`) with
[tier4/autoware_lanelet2_to_opendrive] v2.62.0, targeting CARLA, using the origin above and
left-hand traffic. No manual edits were made to the network: every merge, move, delete and remove
operation the converter offers was left empty. Being derived from a file in this repository, it
carries the same Apache-2.0 license.

[tier4/autoware_lanelet2_to_opendrive]: https://github.com/tier4/autoware_lanelet2_to_opendrive
