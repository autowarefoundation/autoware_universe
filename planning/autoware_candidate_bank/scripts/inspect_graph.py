#!/usr/bin/env python3

# Copyright 2026 TIER IV, Inc.
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

"""Inspect live wiring without changing any running node."""

import time

import rclpy
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("candidate_bank_wiring_inspector")
    end = time.monotonic() + 2
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    banks = [
        (name, ns)
        for name, ns in node.get_node_names_and_namespaces()
        if name == "candidate_bank_node"
    ]
    if not banks:
        raise RuntimeError(
            "No candidate_bank_node found. The top-level launch may bypass diffusion_planner.launch.xml."
        )
    bad = False
    for name, ns in banks:
        print("BANK", ns.rstrip("/") + "/" + name)
        inputs = node.get_subscriber_names_and_types_by_node(name, ns)
        outputs = node.get_publisher_names_and_types_by_node(name, ns)
        for topic, types in inputs:
            if "/parameter_events" == topic:
                continue
            pubs = node.get_publishers_info_by_topic(topic)
            print(
                "INPUT",
                topic,
                types,
                "publishers=",
                [p.node_namespace.rstrip("/") + "/" + p.node_name for p in pubs],
            )
            if (
                any("CandidateTrajectories" in typ or "PredictedObjects" in typ for typ in types)
                and not pubs
            ):
                bad = True
        for topic, types in outputs:
            if not any(
                "CandidateTrajectories" in typ or typ == "autoware_planning_msgs/msg/Trajectory"
                for typ in types
            ):
                continue
            pubs = node.get_publishers_info_by_topic(topic)
            subs = node.get_subscriptions_info_by_topic(topic)
            print(
                "OUTPUT",
                topic,
                types,
                "publishers=",
                len(pubs),
                "subscribers=",
                [p.node_namespace.rstrip("/") + "/" + p.node_name for p in subs],
            )
            if len(pubs) != 1:
                bad = True
                print("ERROR: output must have exactly one publisher")
            if any("CandidateTrajectories" in typ for typ in types) and not subs:
                bad = True
                print("ERROR: selected candidates have no downstream subscriber")
    node.destroy_node()
    rclpy.shutdown()
    if bad:
        raise SystemExit(1)
    print(
        "[OK] Publisher ownership and downstream graph checked. Inspect status for timestamp pairing."
    )


if __name__ == "__main__":
    main()
