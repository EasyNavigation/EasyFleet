// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the projects Arquimea-URJC and AURORAS
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

#include "arch_mockup/detail/param_utils.hpp"

#include <algorithm>
#include <cctype>

namespace arch_mockup
{
namespace detail
{

std::string sanitize_parameter_name(const std::string & action_name)
{
  std::string name = action_name;
  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  std::replace(name.begin(), name.end(), '/', '.');
  return name.empty() ? std::string("action") : name;
}

std::string sanitize_identifier(const std::string & text)
{
  std::string result = text;
  for (auto & c : result) {
    if (!std::isalnum(static_cast<unsigned char>(c))) {
      c = '_';
    }
  }
  if (result.empty() || std::isdigit(static_cast<unsigned char>(result.front()))) {
    result.insert(result.begin(), '_');
  }
  return result;
}

}  // namespace detail
}  // namespace arch_mockup
