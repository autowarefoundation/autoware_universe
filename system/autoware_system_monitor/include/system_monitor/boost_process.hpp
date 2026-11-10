// Copyright 2026 The Autoware Contributors
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

/**
 * @file boost_process.hpp
 * @brief Boost.Process v1 API on every Boost version
 */

#ifndef SYSTEM_MONITOR__BOOST_PROCESS_HPP_
#define SYSTEM_MONITOR__BOOST_PROCESS_HPP_

// Boost 1.88 made Boost.Process v2 the default; v1 stays available under boost/process/v1/
#if __has_include(<boost/process/v1/child.hpp>)
#ifndef BOOST_PROCESS_VERSION
#define BOOST_PROCESS_VERSION 1
#endif
#if __has_include(<boost/process/v1.hpp>)
#include <boost/process/v1.hpp>
#else  // Boost 1.88 has the v1 headers but not v1.hpp
#include <boost/process/v1/args.hpp>
#include <boost/process/v1/async.hpp>
#include <boost/process/v1/async_system.hpp>
#include <boost/process/v1/child.hpp>
#include <boost/process/v1/cmd.hpp>
#include <boost/process/v1/env.hpp>
#include <boost/process/v1/environment.hpp>
#include <boost/process/v1/error.hpp>
#include <boost/process/v1/exe.hpp>
#include <boost/process/v1/group.hpp>
#include <boost/process/v1/handles.hpp>
#include <boost/process/v1/io.hpp>
#include <boost/process/v1/pipe.hpp>
#include <boost/process/v1/search_path.hpp>
#include <boost/process/v1/shell.hpp>
#include <boost/process/v1/spawn.hpp>
#include <boost/process/v1/start_dir.hpp>
#include <boost/process/v1/system.hpp>
#endif
#else
#include <boost/process.hpp>
#endif

#endif  // SYSTEM_MONITOR__BOOST_PROCESS_HPP_
