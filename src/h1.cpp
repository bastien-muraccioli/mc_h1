#include "h1.h"
#include "config.h"

#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

H1RobotModule::H1RobotModule()
  : RobotModule(mc_rtc::H1_DESCRIPTION_PATH,
                "h1",
                std::string{mc_rtc::H1_DESCRIPTION_PATH} + std::string{"/urdf/h1.urdf"})
{
  rsdf_dir = std::string(mc_rtc::H1_DESCRIPTION_PATH) + "/rsdf";
  
  // True if the robot has a fixed base, false otherwise  
  bool fixed = false;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));
  
#if 0
  // Build _convexHull, but conflict primitives defined in h1.urdf
  bfs::path convexPath = bfs::path(path) / "convex/h1";
  for(const auto & b : mb.bodies())
  {
    bfs::path ch = convexPath / (b.name() + ".txt");
    if(bfs::exists(ch))
    {
      _convexHull[b.name()] = {b.name(), ch.string()};
    }
  }
#endif
  
  _ref_joint_order = {
    "left_hip_yaw_joint",
    "left_hip_roll_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_ankle_joint",
    "right_hip_yaw_joint",
    "right_hip_roll_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_ankle_joint",
    "torso_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint"
  };
  
  using namespace mc_rtc::constants;
  _stance["left_hip_yaw_joint"] = {0.0};
  _stance["left_hip_roll_joint"] = {0.0};
  _stance["left_hip_pitch_joint"] = {-0.4};
  _stance["left_knee_joint"] = {0.8};
  _stance["left_ankle_joint"] = {-0.4};
  _stance["right_hip_yaw_joint"] = {0.0};
  _stance["right_hip_roll_joint"] = {0.0};
  _stance["right_hip_pitch_joint"] = {-0.4};
  _stance["right_knee_joint"] = {0.8};
  _stance["right_ankle_joint"] = {-0.4};
  _stance["torso_joint"] = {0.0};
  _stance["left_shoulder_pitch_joint"] = {0.0};
  _stance["left_shoulder_roll_joint"] = {0.0};
  _stance["left_shoulder_yaw_joint"] = {0.0};
  _stance["left_elbow_joint"] = {0.0};
  _stance["right_shoulder_pitch_joint"] = {0.0};
  _stance["right_shoulder_roll_joint"] = {0.0};
  _stance["right_shoulder_yaw_joint"] = {0.0};
  _stance["right_elbow_joint"] = {0.0};
  
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.837656}};
  
  // Add JointSensors for temperature/current logging
  for(size_t i = 0; i < _ref_joint_order.size(); ++i)
  {
    if(mb.jointIndexByName().count(_ref_joint_order[i]) != 0)
    {
      _jointSensors.push_back(mc_rbdyn::JointSensor(_ref_joint_order[i]));
    }
  }
  
  // Sensors
  _bodySensors.clear();
  _bodySensors.emplace_back("Accelerometer", "pelvis", sva::PTransformd(Eigen::Vector3d(-0.04452, -0.01891, 0.27756)));
  _bodySensors.emplace_back("FloatingBase", "pelvis", sva::PTransformd::Identity());
  
  _minimalSelfCollisions = {
    mc_rbdyn::Collision("torso_link", "left_shoulder_yaw_link", 0.02, 0.001, 0.),
    mc_rbdyn::Collision("torso_link", "right_shoulder_yaw_link", 0.02, 0.001, 0.),
    mc_rbdyn::Collision("torso_link", "left_elbow_link", 0.05, 0.03, 0.),
    mc_rbdyn::Collision("torso_link", "right_elbow_link", 0.05, 0.03, 0.),
    mc_rbdyn::Collision("pelvis", "left_shoulder_yaw_link", 0.05, 0.03, 0.),
    mc_rbdyn::Collision("pelvis", "right_shoulder_yaw_link", 0.05, 0.03, 0.),
    mc_rbdyn::Collision("pelvis", "left_elbow_link", 0.05, 0.03, 0.),
    mc_rbdyn::Collision("pelvis", "right_elbow_link", 0.05, 0.03, 0.),
    mc_rbdyn::Collision("left_hip_pitch_link", "right_hip_pitch_link", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("left_knee_link", "right_knee_link", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("left_ankle_link", "right_ankle_link", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("left_ankle_link", "right_knee_link", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("right_ankle_link", "left_knee_link", 0.02, 0.01, 0.)};
  _commonSelfCollisions = _minimalSelfCollisions;
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"H1"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("H1")
    if(n == "H1")
    {
      return new mc_robots::H1RobotModule();
    }
    else
    {
      mc_rtc::log::error("H1 module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
