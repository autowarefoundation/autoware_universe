# Scene Understanding Result Message
# Generated from ONNX model output

std_msgs/Header header

# Exclusive Categories (single choice per category)
string intersection              # T-junction, No intersection, 4-way intersection
string lane_position            # Special lane, Center lane, Left lane, Right lane
string lighting                 # Daytime, Glare conditions
string road_geometry            # Straight, Curved, Slope
string road_surface             # Dry, Degraded
string road_type                # Urban street, Residential area, Bridge
string traffic_density          # Empty, Light, Moderate, Heavy
string visibility               # Clear, Partially obstructed, Severely limited
string weather                  # Cloudy

# Binary Categories (probability values 0-1)
float32 road_features_construction_zone
float32 road_features_crosswalk
float32 road_features_merge_area
float32 road_features_railway_crossing
float32 special_vehicles_emergency_vehicle
float32 special_vehicles_heavy_vehicle
float32 special_vehicles_public_transport
float32 traffic_control_no_traffic_controls
float32 traffic_control_stop_sign
float32 traffic_control_traffic_light
float32 traffic_control_yield_sign
float32 vulnerable_road_users_animals
float32 vulnerable_road_users_cyclists
float32 vulnerable_road_users_pedestrians
