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

#include "easyfleet_navigation_manager/data_path.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"

namespace easyfleet
{

std::string resolve_data_path(
  const std::string & package, const std::string & file, const std::string & prefix)
{
  if (file.empty()) {
    throw std::runtime_error("Parameter '" + prefix + ".map_path_file' must be set.");
  }
  const std::filesystem::path path(file);
  if (path.is_absolute()) {
    return path.string();
  }
  if (package.empty()) {
    throw std::runtime_error(
            "Parameter '" + prefix + ".package' must be set when '" + prefix +
            ".map_path_file' is a relative path.");
  }
  return (ament_index_cpp::get_package_share_path(package) / path).string();
}

}  // namespace easyfleet
