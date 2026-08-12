// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project EasyFleet
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

#ifndef EASYFLEET_MISSION_MANAGER__OUTPUT_HPP_
#define EASYFLEET_MISSION_MANAGER__OUTPUT_HPP_

#include <iostream>
#include <mutex>
#include <string>

namespace easyfleet_mission_manager
{

/// Serializes terminal output across the threads used to run capabilities
/// in parallel, so concurrent feedback lines never get interleaved
/// character-by-character.
inline std::mutex g_output_mutex;

inline void safe_print(const std::string & line)
{
  std::lock_guard<std::mutex> lock(g_output_mutex);
  std::cout << line << std::endl;
}

}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__OUTPUT_HPP_
