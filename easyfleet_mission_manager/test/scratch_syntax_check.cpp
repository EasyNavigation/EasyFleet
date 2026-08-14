// Temporary: verifies the not-yet-implemented RobotHandle/FleetSession/
// SimpleController headers at least parse and type-check. FleetSession has
// no method bodies yet, so nothing here is actually called -- to be
// replaced by real tests once they're implemented.

#include "easyfleet_mission_manager/capability_state.hpp"
#include "easyfleet_mission_manager/fleet_session.hpp"
#include "easyfleet_mission_manager/robot_handle.hpp"
#include "easyfleet_mission_manager/simple_controller.hpp"

#include "gtest/gtest.h"

void unused_reference_check(
  easyfleet::RobotHandle & robot,
  easyfleet::FleetSession & session,
  easyfleet::SimpleController & controller)
{
  (void)robot;
  (void)session;
  (void)controller;
}

TEST(ScratchSyntaxCheck, HeadersParse)
{
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
