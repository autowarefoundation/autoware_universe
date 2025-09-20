// Copyright 2023 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "config/errors.hpp"
#include "graph/graph.hpp"
#include "tests/utils.hpp"

#include <gtest/gtest.h>

using namespace autoware::diagnostic_graph_aggregator;  // NOLINT(build/namespaces)

TEST(GraphLoad, RootNotFound)
{
  EXPECT_THROW(Graph(resource("graph-load/fake-file-name.yaml")), FileNotFound);
}

TEST(GraphLoad, FileNotFound)
{
  EXPECT_THROW(Graph(resource("graph-load/file-not-found.yaml")), FileNotFound);
}

TEST(GraphLoad, InvalidDictType)
{
  EXPECT_THROW(Graph(resource("graph-load/invalid-dict-type.yaml")), InvalidType);
}

TEST(GraphLoad, InvalidListType)
{
  EXPECT_THROW(Graph(resource("graph-load/invalid-list-type.yaml")), InvalidType);
}

TEST(GraphLoad, FieldNotFound)
{
  EXPECT_THROW(Graph(resource("graph-load/field-not-found.yaml")), FieldNotFound);
}

TEST(GraphLoad, UnknownSubstitution)
{
  EXPECT_THROW(Graph(resource("graph-load/unknown-substitution.yaml")), UnknownSubstitution);
}

TEST(GraphLoad, UnknownLogic)
{
  EXPECT_THROW(Graph(resource("graph-load/unknown-logic-type.yaml")), UnknownLogic);
}

TEST(GraphLoad, PathConflict)
{
  EXPECT_THROW(Graph(resource("graph-load/path-conflict.yaml")), PathConflict);
}

TEST(GraphLoad, PathNotFound)
{
  EXPECT_THROW(Graph(resource("graph-load/path-not-found.yaml")), PathNotFound);
}

TEST(GraphLoad, LinkLoopFound)
{
  EXPECT_THROW(Graph(resource("graph-load/link-loop.yaml")), LinkLoopFound);
}

TEST(GraphLoad, UnitLoopFound)
{
  EXPECT_THROW(Graph(resource("graph-load/unit-loop.yaml")), UnitLoopFound);
}

TEST(GraphLoad, RemoveUnknownUnitByEdit)
{
  EXPECT_THROW(Graph(resource("graph-load/remove-unknown-unit-by-edit.yaml")), PathNotFound);
}

TEST(GraphLoad, RemoveUnknownUnitByRegexEdit)
{
  EXPECT_THROW(Graph(resource("graph-load/remove-unknown-unit-by-regex-edit.yaml")), PathNotFound);
}
