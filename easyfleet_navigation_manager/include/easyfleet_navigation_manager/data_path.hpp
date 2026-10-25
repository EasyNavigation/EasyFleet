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

#ifndef EASYFLEET_NAVIGATION_MANAGER__DATA_PATH_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__DATA_PATH_HPP_

#include <string>

namespace easyfleet
{

/// @brief Full path of a map/routes file given as a `<prefix>.package` /
/// `<prefix>.map_path_file` parameter pair: `file` relative to the
/// package's share directory, or `file` itself when it is absolute (then
/// `package` may be empty, e.g. for maps kept outside any package).
/// @param package Package name, or empty for an absolute `file`.
/// @param file File path, relative to the package share or absolute.
/// @param prefix Parameter prefix, for error messages (e.g. "map").
/// @return The resolved path.
/// @throw std::runtime_error if `file` is empty, or relative with no
///   `package`, or the package doesn't exist.
std::string resolve_data_path(
  const std::string & package, const std::string & file, const std::string & prefix);

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__DATA_PATH_HPP_
