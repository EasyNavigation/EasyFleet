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

// Test-only pluginlib CapabilityFactory, registered so easyfleet_core's own
// gtest suite can exercise Deployment's plugin-loading success path
// (add_capability() -> start() -> ... ) without depending on a downstream
// deployment package. Built as its own shared library and registered via
// test_plugins.xml / pluginlib_export_plugin_description_file(), exactly
// like a real deployment package's own CapabilityFactory plugin.

#include "pluginlib/class_list_macros.hpp"

#include "easyfleet_core/capability_factory.hpp"

#include "test_capability_server.hpp"

namespace easyfleet_core_test
{

using TestCapabilityFactory = easyfleet_core::CapabilityFactoryFor<TestCapabilityActionServer>;

}  // namespace easyfleet_core_test

PLUGINLIB_EXPORT_CLASS(
  easyfleet_core_test::TestCapabilityFactory,
  easyfleet_core::CapabilityFactory)
