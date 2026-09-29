#!/usr/bin/env python3
"""Regenerate dummy_network.onnx, the fixture used by tensorrt_common_test.

The graph is the smallest one that can exercise IO reconciliation: three dynamically
shaped inputs that all feed the single output, so TensorRT declares every one of them.

Writes the file next to this script, so it can be run from anywhere.

Usage: python3 generate_dummy_network.py
"""

# cspell:ignore opset, opsetid

import pathlib

import onnx
from onnx import TensorProto
from onnx import helper

DIM = "num_rows"
WIDTH = 4


def tensor(name):
    return helper.make_tensor_value_info(name, TensorProto.FLOAT, [DIM, WIDTH])


graph = helper.make_graph(
    nodes=[
        helper.make_node("Add", ["required_a", "required_b"], ["partial"]),
        helper.make_node("Add", ["partial", "optional_present"], ["sum"]),
    ],
    name="dummy_network",
    inputs=[tensor("required_a"), tensor("required_b"), tensor("optional_present")],
    outputs=[tensor("sum")],
)

model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
model.ir_version = 9
onnx.checker.check_model(model)
onnx.save(model, str(pathlib.Path(__file__).with_name("dummy_network.onnx")))
print("wrote dummy_network.onnx")
