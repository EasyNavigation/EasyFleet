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

#include "easyfleet_mission_manager/capability_info.hpp"

#include <sstream>
#include <string>

#include "easyfleet_mission_manager/ansi.hpp"
#include "easyfleet_mission_manager/output.hpp"

namespace easyfleet_mission_manager
{

namespace
{

std::string get_string(
  const nlohmann::json & j, const std::string & key,
  const std::string & fallback = "")
{
  if (j.contains(key) && j[key].is_string()) {
    return j[key].get<std::string>();
  }
  return fallback;
}

void print_bullet_list(std::ostringstream & out, const nlohmann::json & j, const std::string & key)
{
  if (!j.contains(key) || !j[key].is_array()) {
    return;
  }
  out << "\n  " << ansi::bold << key << ":" << ansi::reset << "\n";
  for (const auto & item : j[key]) {
    if (item.is_string()) {
      out << "    " << ansi::dim << "-" << ansi::reset << " " << item.get<std::string>() << "\n";
    }
  }
}

std::string identity(const CapabilityInfo & info)
{
  return info.robot.empty() ? info.capability : info.robot + "/" + info.capability;
}

/// Three-state status label: a capability that isn't heartbeating at all is
/// "INACTIVE"; one that is, is either "BUSY" (a goal is currently
/// executing) or "IDLE" (ready to accept one) -- the distinction an
/// orchestrator needs before deciding what to call next.
std::string status_label(const CapabilityInfo & info)
{
  if (!info.active) {
    return std::string(ansi::red) + "INACTIVE" + ansi::reset;
  }
  if (info.busy) {
    return std::string(ansi::yellow) + "BUSY" + ansi::reset;
  }
  return std::string(ansi::green) + "IDLE" + ansi::reset;
}

}  // namespace

void print_capability_summary_line(const CapabilityInfo & info)
{
  const std::string status = status_label(info);
  const std::string display_name =
    info.description_json_valid ?
    get_string(info.description_json, "display_name", info.capability) :
    info.capability;

  std::ostringstream out;
  out << "  " << ansi::bold << identity(info) << ansi::reset << " - " << display_name
      << "  (" << ansi::dim << info.action_name << ansi::reset << ")  [" << status << "]";
  safe_print(out.str());
}

void print_capability_info(const CapabilityInfo & info)
{
  std::ostringstream out;

  const std::string display_name =
    info.description_json_valid ?
    get_string(info.description_json, "display_name", info.capability) :
    info.capability;
  const std::string status = status_label(info);

  const std::string title = identity(info) + " -- " + display_name;
  const std::string bar(title.size() + 4, '=');

  out << ansi::bold << ansi::cyan << bar << "\n  " << title << "   [" << status << ansi::cyan <<
    "]\n"
      << bar << ansi::reset << "\n";
  out << "\n  " << ansi::bold << "Action:" << ansi::reset << " " << info.action_name << "\n";

  if (!info.description_json_valid) {
    out << ansi::red << "  (could not parse the JSON description published on /capabilities)\n"
        << ansi::reset << ansi::dim << info.description_json_raw << ansi::reset;
    safe_print(out.str());
    return;
  }

  out << "\n  " << get_string(info.description_json, "description", "(no description)") << "\n";

  if (info.description_json.contains("action") && info.description_json["action"].is_object()) {
    const auto & action = info.description_json["action"];
    out << "\n  " << ansi::bold << "Action type:" << ansi::reset << " "
        << get_string(action, "type") << "\n";
  }

  print_bullet_list(out, info.description_json, "requirements");
  print_bullet_list(out, info.description_json, "effects");
  print_bullet_list(out, info.description_json, "notes");

  if (info.description_json.contains("parameters") &&
    info.description_json["parameters"].is_object())
  {
    out << "\n  " << ansi::bold << "Parameters:" << ansi::reset << "\n";
    for (const auto & [param_name, param_info] : info.description_json["parameters"].items()) {
      out << "    " << ansi::yellow << param_name << ansi::reset;
      if (param_info.is_object() && param_info.contains("type")) {
        out << " (" << get_string(param_info, "type") << ")";
      }
      out << "\n";
      if (param_info.is_object() && param_info.contains("description")) {
        out << "        " << ansi::dim << get_string(param_info,
            "description") << ansi::reset << "\n";
      }
    }
  }

  safe_print(out.str());
}

}  // namespace easyfleet_mission_manager
