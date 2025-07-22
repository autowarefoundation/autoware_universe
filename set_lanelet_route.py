#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autoware_planning_msgs.srv import SetLaneletRoute
from autoware_planning_msgs.msg import LaneletSegment, LaneletPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from unique_identifier_msgs.msg import UUID
import uuid

def make_uuid():
    u = uuid.uuid4()
    return UUID(uuid=list(u.bytes))

def main():
    rclpy.init()
    node = rclpy.create_node('set_lanelet_route_manual')

    cli = node.create_client(SetLaneletRoute, '/planning/set_lanelet_route')
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available.')
        return

    req = SetLaneletRoute.Request()

    # Fill header (optional)
    req.header = Header()
    req.header.frame_id = 'map'

    # Fill goal_pose
    req.goal_pose = Pose()
    req.goal_pose.position.x = 88956.6
    req.goal_pose.position.y = 42538.0
    req.goal_pose.position.z = 0.0
    req.goal_pose.orientation.x = 0.0
    req.goal_pose.orientation.y = 0.0
    req.goal_pose.orientation.z = 0.882127
    req.goal_pose.orientation.w = 0.471011

    # Fill UUID
    req.uuid = make_uuid()

    # Fill allow_modification
    req.allow_modification = False

    # Manually define a few dummy segments (lanelet IDs must be valid!)
    lanelet_primitive1 = LaneletPrimitive()
    lanelet_primitive1.id = 177179
    lanelet_primitive1.primitive_type = 'lane'  # or 'crosswalk' etc.

    lanelet_primitive2 = LaneletPrimitive()
    lanelet_primitive2.id = 178745
    lanelet_primitive2.primitive_type = 'lane'  # or 'crosswalk' etc.

    lanelet_primitive3 = LaneletPrimitive()
    lanelet_primitive3.id = 178750
    lanelet_primitive3.primitive_type = 'lane'  # or 'crosswalk' etc.
    
    segment1 = LaneletSegment()
    segment1.preferred_primitive = lanelet_primitive2
    segment1.primitives = [lanelet_primitive1, lanelet_primitive2, lanelet_primitive3]

    lanelet_primitive4 = LaneletPrimitive()
    lanelet_primitive4.id = 177510
    lanelet_primitive4.primitive_type = 'lane'  # or 'crosswalk' etc.

    lanelet_primitive5 = LaneletPrimitive()
    lanelet_primitive5.id = 177879
    lanelet_primitive5.primitive_type = 'lane'  # or 'crosswalk' etc.

    lanelet_primitive6 = LaneletPrimitive()
    lanelet_primitive6.id = 178755
    lanelet_primitive6.primitive_type = 'lane'  # or 'crosswalk' etc.

    segment2 = LaneletSegment()
    segment2.preferred_primitive = lanelet_primitive5
    segment2.primitives = [lanelet_primitive4, lanelet_primitive5, lanelet_primitive6]

    lanelet_primitive7 = LaneletPrimitive()
    lanelet_primitive7.id = 123
    lanelet_primitive7.primitive_type = 'lane'  # or 'crosswalk' etc.

    lanelet_primitive8 = LaneletPrimitive()
    lanelet_primitive8.id = 124
    lanelet_primitive8.primitive_type = 'lane'  # or 'crosswalk' etc.

    lanelet_primitive9 = LaneletPrimitive()
    lanelet_primitive9.id = 125
    lanelet_primitive9.primitive_type = 'lane'  # or 'crosswalk' etc.
    
    segment3 = LaneletSegment()
    segment3.preferred_primitive = lanelet_primitive8
    segment3.primitives = [lanelet_primitive7, lanelet_primitive8, lanelet_primitive9]

    req.segments = [segment1, segment2, segment3]

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result():
        node.get_logger().info(f"Response: {future.result().status}")
    else:
        node.get_logger().error("Service call failed.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
