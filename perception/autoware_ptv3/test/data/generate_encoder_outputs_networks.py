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

"""Regenerate the networks encoder_outputs_test runs.

encoder_outputs_encoder.onnx maps rows [num_rows, 4] to `feat` [num_rows, 4] and `doubled`
[2 * num_rows, 4], whose shapes follow from the input shape. encoder_outputs_head.onnx reads
both and concatenates them into `out` [3 * num_rows, 4]. encoder_outputs_nonzero.onnx outputs
the nonzero indices of its input, whose shape depends on the input data.

Writes the files next to this script, so it can be run from anywhere.

Usage: python3 generate_encoder_outputs_networks.py
"""

# cspell:ignore opset, opsetid, nonzero

import pathlib

import onnx
from onnx import TensorProto
from onnx import helper

WIDTH = 4


def tensor(name, rows, dtype=TensorProto.FLOAT, width=WIDTH):
    return helper.make_tensor_value_info(name, dtype, [rows, width])


def save(graph, name):
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 9
    onnx.checker.check_model(model)
    onnx.save(model, str(pathlib.Path(__file__).with_name(name)))
    print(f"wrote {name}")


save(
    helper.make_graph(
        nodes=[
            helper.make_node("Relu", ["rows"], ["feat"]),
            helper.make_node("Concat", ["rows", "rows"], ["doubled"], axis=0),
        ],
        name="encoder",
        inputs=[tensor("rows", "num_rows")],
        outputs=[tensor("feat", "num_rows"), tensor("doubled", "doubled_rows")],
    ),
    "encoder_outputs_encoder.onnx",
)
save(
    helper.make_graph(
        nodes=[helper.make_node("Concat", ["feat", "doubled"], ["out"], axis=0)],
        name="head",
        inputs=[tensor("feat", "num_rows"), tensor("doubled", "doubled_rows")],
        outputs=[tensor("out", "out_rows")],
    ),
    "encoder_outputs_head.onnx",
)
save(
    helper.make_graph(
        nodes=[helper.make_node("NonZero", ["rows"], ["nonzero"])],
        name="nonzero",
        inputs=[tensor("rows", "num_rows")],
        outputs=[tensor("nonzero", 2, TensorProto.INT64, "num_nonzero")],
    ),
    "encoder_outputs_nonzero.onnx",
)
