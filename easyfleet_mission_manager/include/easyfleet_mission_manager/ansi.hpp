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

#ifndef EASYFLEET_MISSION_MANAGER__ANSI_HPP_
#define EASYFLEET_MISSION_MANAGER__ANSI_HPP_

namespace easyfleet_mission_manager
{
namespace ansi
{

constexpr const char * reset = "\033[0m";
constexpr const char * bold = "\033[1m";
constexpr const char * dim = "\033[2m";
constexpr const char * red = "\033[31m";
constexpr const char * green = "\033[32m";
constexpr const char * yellow = "\033[33m";
constexpr const char * blue = "\033[34m";
constexpr const char * magenta = "\033[35m";
constexpr const char * cyan = "\033[36m";

}  // namespace ansi
}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__ANSI_HPP_
