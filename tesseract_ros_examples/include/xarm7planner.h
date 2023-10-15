/**
 * @file plotting.h
 * @brief Tesseract ROS Basic plotting functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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


#include "ros/ros.h"
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/publisher.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract_examples/xarm7_trajopt.h>

#include <tesseract_msgs/Xarm7Plan.h>


TESSERACT_COMMON_IGNORE_WARNINGS_POP


namespace tesseract_rosutils
{

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
  tesseract_examples::Xarm7Trajopt xarm7_trajopt_runner_;

};

}  // namespace tesseract_rosutils