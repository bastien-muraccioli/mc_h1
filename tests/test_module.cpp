#include <mc_rbdyn/RobotLoader.h>

#include "h1.h"
#include "test_module.h"

int main()
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({MODULE_DIR});

  using namespace mc_robots;
  H1RobotModule::ForAllVariants([&](const std::string & ee_type_right, const std::string & ee_type_left) {
    auto name = H1RobotModule::NameFromParams(ee_type_right, ee_type_left);
    auto robot = mc_rbdyn::RobotLoader::get_robot_module(name);
    auto mass = [robot]() {
      double mass = 0.0;
      for(const auto & b : robot->mb.bodies())
      {
        mass += b.inertia().mass();
      }
      return mass;
    }();
    mc_rtc::log::info("{} has {} dof, mass: {}", name, robot->mb.nrDof(), mass);
  });

  return 0;
}
