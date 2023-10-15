/**
 * @file xarm7_trajopt_node.cpp
 * @brief Glass upright example node
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_examples/xarm7_trajopt.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/conversions.h>
#include "ros/ros.h"
#include <tesseract_msgs/Xarm7Plan.h>

using namespace tesseract_examples;
using namespace tesseract_rosutils;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";



void init_vectorXd(Eigen::VectorXd& vec, std::vector<double> vec_in)
{
  vec.resize(vec_in.size());
  for (int i = 0; i < vec_in.size(); i++)
  {
    vec(i) = vec_in[i];
  }
}

void convert_deg_to_rad(Eigen::VectorXd& vec)
{
  for (int i = 0; i < vec.size(); i++)
  {
    vec(i) = vec(i) * M_PI / 180.0;
  }
}

void convert_rad_to_deg(Eigen::VectorXd& vec)
{
  for (int i = 0; i < vec.size(); i++)
  {
    vec(i) = vec(i) * 180.0 / M_PI;
  }
}

Eigen::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
Eigen::Vector3d ToEulerAngles(Eigen::Quaterniond q) {
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
    double cosp = std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
    angles(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}



class Xarm7Planner
{
public:
  Xarm7Planner(ros::NodeHandle nh, bool plotting, bool rviz, bool debug,
   std::string urdf_xml_string, std::string srdf_xml_string);


  bool run_planning_service(tesseract_msgs::Xarm7Plan::Request  &req,
                            tesseract_msgs::Xarm7Plan::Response &res);

  bool start();

private:
  ros::NodeHandle nh_;
  bool plotting_;
  bool rviz_;
  bool debug_;
  std::string urdf_xml_string_;
  std::string srdf_xml_string_;
  Xarm7Trajopt xarm7_trajopt_runner_;

};



Xarm7Planner::Xarm7Planner(ros::NodeHandle nh, bool plotting, bool rviz, bool debug, std::string urdf_xml_string, std::string srdf_xml_string)
  : nh_(nh), plotting_(plotting), rviz_(rviz),
   debug_(debug), urdf_xml_string_(urdf_xml_string), srdf_xml_string_(srdf_xml_string)
{
}

bool Xarm7Planner::start()
{
  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string_, srdf_xml_string_, locator))
    exit(1);

  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor->startPublishingEnvironment();

  tesseract_rosutils::ROSPlottingPtr plotter;
  if (plotting_)
    plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(env->getSceneGraph()->getRoot());  

  Xarm7Trajopt example(env, plotter);
  xarm7_trajopt_runner_ = example;

  ros::ServiceServer service = nh_.advertiseService('xarm7plan', &Xarm7Planner::run_planning_service, this);

  ros::spin();


  return 1;
}

bool Xarm7Planner::run_planning_service(tesseract_msgs::Xarm7Plan::Request  &req,
                                        tesseract_msgs::Xarm7Plan::Response &res)
{
  ROS_INFO("handling planning request");

  Eigen::VectorXd joint_start_pos(7);
  init_vectorXd(joint_start_pos, req.initial_joint_state);
  convert_deg_to_rad(joint_start_pos);


  std::vector<double> final_pose_euler;
  Eigen::VectorXd final_pose_euler_eig(3);
  init_vectorXd(final_pose_euler_eig, req.final_pose_euler);
  std::cout << "final_pose_euler_eig "<< final_pose_euler_eig << std::endl;
  convert_deg_to_rad(final_pose_euler_eig);
  std::cout << "final_pose_euler_eig rad "<< final_pose_euler_eig << std::endl;


  std::vector<double> final_pose_xyz;
  Eigen::Vector3d final_pose_xyz_eig(req.final_pose_xyz);

  Eigen::Quaterniond quat_eig;
  quat_eig = EulerToQuaternion(final_pose_euler_eig(0), final_pose_euler_eig(1), final_pose_euler_eig(2));
  std::cout << "quat_eig vec " << quat_eig.x() << quat_eig.y() << quat_eig.z() << std::endl;
  std::cout << "quat_eig w " << quat_eig.w() << std::endl;

  Eigen::Vector3d euler_eig = ToEulerAngles(quat_eig);
  std::cout << "convert back to euler "<< euler_eig(0)*180/M_PI << euler_eig(1)*180/M_PI << euler_eig(2)*180/M_PI << std::endl;
  
  Eigen::Isometry3d final_pose_eig;
  final_pose_eig.linear() = quat_eig.matrix();
  final_pose_eig.translation() = final_pose_xyz_eig;

  std::cout << "node final_pose_ rotation "<< final_pose_eig.linear() << std::endl;
  std::cout << "node final_pose_ translation "<< final_pose_eig.translation() << std::endl;

  
  int success = 0;
  success = xarm7_trajopt_runner_.run(joint_start_pos, final_pose_eig);
  ROS_INFO("success: %d", success);

  return true;
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm7_trajopt_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  bool debug = false;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("debug", debug, debug);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  Xarm7Planner planner(nh, plotting, rviz, debug, urdf_xml_string, srdf_xml_string);
  planner.start();
}


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "xarm7_trajopt_node");
//   ros::NodeHandle pnh("~");
//   ros::NodeHandle nh;

//   bool plotting = true;
//   bool rviz = true;
//   bool ifopt = false;
//   bool debug = false;

//   // Get ROS Parameters
//   pnh.param("plotting", plotting, plotting);
//   pnh.param("rviz", rviz, rviz);
//   pnh.param("ifopt", ifopt, ifopt);
//   pnh.param("debug", debug, debug);

//   std::vector<double> arm_start;
//   pnh.param("arm_start", arm_start, arm_start);
//   Eigen::VectorXd arm_start_eig(arm_start.size());
//   init_vectorXd(arm_start_eig, arm_start);
//   convert_deg_to_rad(arm_start_eig);


//   std::vector<double> arm_end;
//   pnh.param("arm_end", arm_end, arm_end);
//   Eigen::VectorXd arm_end_eig(arm_end.size());
//   init_vectorXd(arm_end_eig, arm_end);
//   convert_deg_to_rad(arm_end_eig);  


//   std::vector<double> final_pose_euler;
//   pnh.param("final_pose_euler", final_pose_euler, final_pose_euler);
//   Eigen::VectorXd final_pose_euler_eig(final_pose_euler.size());
//   init_vectorXd(final_pose_euler_eig, final_pose_euler);
//   std::cout << "final_pose_euler_eig "<< final_pose_euler_eig << std::endl;
//   convert_deg_to_rad(final_pose_euler_eig);
//   std::cout << "final_pose_euler_eig rad "<< final_pose_euler_eig << std::endl;


//   std::vector<double> final_pose_xyz;
//   pnh.param("final_pose_xyz", final_pose_xyz, final_pose_xyz);
//   Eigen::Vector3d final_pose_xyz_eig(final_pose_xyz.data());

//   Eigen::Quaterniond quat_eig;
//   quat_eig = EulerToQuaternion(final_pose_euler_eig(0), final_pose_euler_eig(1), final_pose_euler_eig(2));
//   std::cout << "quat_eig vec " << quat_eig.x() << quat_eig.y() << quat_eig.z() << std::endl;
//   std::cout << "quat_eig w " << quat_eig.w() << std::endl;

//   Eigen::Vector3d euler_eig = ToEulerAngles(quat_eig);
//   std::cout << "convert back to euler "<< euler_eig(0)*180/M_PI << euler_eig(1)*180/M_PI << euler_eig(2)*180/M_PI << std::endl;
  
//   Eigen::Isometry3d final_pose_eig;
//   final_pose_eig.linear() = quat_eig.matrix();
//   final_pose_eig.translation() = final_pose_xyz_eig;

  
//   std::cout << "node final_pose_ rotation "<< final_pose_eig.linear() << std::endl;
//   std::cout << "node final_pose_ translation "<< final_pose_eig.translation() << std::endl;


//   if (ifopt == true)
//   {
//     ROS_INFO("Using TrajOpt Ifopt!");
//   }

//   // Initial setup
//   std::string urdf_xml_string, srdf_xml_string;
//   nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
//   nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

//   auto env = std::make_shared<tesseract_environment::Environment>();
//   auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
//   if (!env->init(urdf_xml_string, srdf_xml_string, locator))
//     exit(1);

//   // Create monitor
//   auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
//   if (rviz)
//     monitor->startPublishingEnvironment();

//   ROSPlottingPtr plotter;
//   if (plotting)
//     plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());

//   Xarm7Trajopt example(env, plotter, ifopt, debug, arm_start_eig, arm_end_eig, final_pose_eig);
//   example.run();
// }
