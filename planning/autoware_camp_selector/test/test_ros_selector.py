"""Real ROS node check; requires an existing compiled Autoware/ROS underlay."""

import json
import os
import subprocess
import time
from pathlib import Path

import rclpy
from autoware_camp_selector.msg import CampCandidatePool, CampSelection
from autoware_internal_planning_msgs.msg import CandidateTrajectory
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand


def spin_until(node, predicate, seconds=10):
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate(), "ROS node/message did not become ready before deadline"


def test_nonzero_original_row_and_malformed_pool_rejection():
    model_path = Path(__file__).resolve().parents[1] / "config/camp_v26_k8_50k.json"
    model = json.loads(model_path.read_text())
    namespace = "/camp_selector_test_" + str(os.getpid())
    rclpy.init()
    node = rclpy.create_node("observer", namespace=namespace)
    selected, trajectories, turns = [], [], []
    subscriptions = [
        node.create_subscription(CampSelection, namespace + "/camp_selector/output/selection", selected.append, 1),
        node.create_subscription(Trajectory, namespace + "/camp_selector/output/trajectory", trajectories.append, 1),
        node.create_subscription(TurnIndicatorsCommand, namespace + "/camp_selector/output/turn_indicators", turns.append, 1),
    ]
    publisher = node.create_publisher(CampCandidatePool, namespace + "/camp_selector/input/candidate_pool", 1)
    process = subprocess.Popen([
        os.environ["CAMP_SELECTOR_EXECUTABLE"], "--ros-args",
        "-r", "__ns:=" + namespace, "-p", "fixed_weight_model_path:=" + str(model_path),
    ])
    try:
        spin_until(node, lambda: publisher.get_subscription_count() == 1)
        spin_until(node, lambda: all(node.count_publishers(s.topic_name) == 1 for s in subscriptions))
        pool = CampCandidatePool()
        pool.header.stamp.sec = 10
        pool.header.frame_id = "map"
        pool.pool_id = 42
        pool.atom_status = [CampCandidatePool.OBSERVED] * 16
        for row in range(model["candidate_pool_k"]):
            candidate = CandidateTrajectory()
            candidate.header = pool.header
            for x in (row + 1.0, row + 2.0):
                point = TrajectoryPoint()
                point.pose.position.x = x
                point.pose.orientation.w = 1.0
                candidate.points.append(point)
            candidate.turn_indicators_command.stamp = pool.header.stamp
            candidate.turn_indicators_command.command = TurnIndicatorsCommand.ENABLE_LEFT
            pool.candidates.candidate_trajectories.append(candidate)
            pool.raw_atoms.extend(model["scales"] if row != 7 else [0.0] * 16)
        publisher.publish(pool)
        spin_until(node, lambda: len(selected) == len(trajectories) == len(turns) == 1)
        assert selected[0].selected_index == 7
        assert selected[0].header == pool.header
        assert selected[0].pool_id == pool.pool_id
        assert selected[0].candidate == pool.candidates.candidate_trajectories[7]
        assert trajectories[0].points == pool.candidates.candidate_trajectories[7].points
        assert turns[0] == pool.candidates.candidate_trajectories[7].turn_indicators_command
        assert len(selected[0].costs) == model["candidate_pool_k"]
        pool.raw_atoms.pop()
        publisher.publish(pool)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(selected) == len(trajectories) == len(turns) == 1
        assert process.poll() is None
        pool.raw_atoms.append(0.0)
        pool.header.stamp.sec = 11
        pool.pool_id = 43
        for candidate in pool.candidates.candidate_trajectories:
            candidate.header = pool.header
            candidate.turn_indicators_command.stamp = pool.header.stamp
        publisher.publish(pool)
        spin_until(node, lambda: len(selected) == len(trajectories) == len(turns) == 2)
        assert selected[-1].header.stamp.sec == 11
        assert selected[-1].selected_index == 7
    finally:
        process.terminate()
        process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
