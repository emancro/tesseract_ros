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
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>

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



int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm7_trajopt_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  bool ifopt = false;
  bool debug = false;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("ifopt", ifopt, ifopt);
  pnh.param("debug", debug, debug);

  std::vector<double> sphere1;
  pnh.param("sphere1", sphere1, sphere1);
  Eigen::Vector3d sphere1_eigen(sphere1.data());

  std::vector<double> arm_start;
  pnh.param("arm_start", arm_start, arm_start);
  Eigen::VectorXd arm_start_eig(arm_start.size());
  init_vectorXd(arm_start_eig, arm_start);
  convert_deg_to_rad(arm_start_eig);


  std::vector<double> arm_end;
  pnh.param("arm_end", arm_end, arm_end);
  Eigen::VectorXd arm_end_eig(arm_end.size());
  init_vectorXd(arm_end_eig, arm_end);
  convert_deg_to_rad(arm_end_eig);  


  if (ifopt == true)
  {
    ROS_INFO("Using TrajOpt Ifopt!");
  }

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator))
    exit(1);

  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz)
    monitor->startPublishingEnvironment();

  ROSPlottingPtr plotter;
  if (plotting)
    plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());

  Xarm7Trajopt example(env, plotter, ifopt, debug, sphere1_eigen, arm_start_eig, arm_end_eig);
  example.run();
}
