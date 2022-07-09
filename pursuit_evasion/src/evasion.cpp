/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(2.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
  double desired_yaw = 0.0;
  double evasion_theta = 0.0;
  bool random_evasion=false;
  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);
  nh_private.param("random_evasion", random_evasion, random_evasion);

  nh_private.param("evasion_theta", evasion_theta, evasion_theta);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
    
  ros::Duration(2.0).sleep();

  float evasion_velocity=0.5;               // evasion velocity  m/s

  // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
  //          nh.getNamespace().c_str(), desired_position.x(),
  //          desired_position.y(), desired_position.z());

  trajectory_pub.publish(trajectory_msg);

  ros::Duration(1.0).sleep();

  mav_msgs::EigenTrajectoryPoint traj_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  traj_point.position_W.x()=desired_position.x();
  traj_point.position_W.y()=desired_position.y();
  traj_point.position_W.z()=desired_position.z();
  
  float dt=0.5;
  static int n_seq = 0;
  while ( ros::ok())
  {


    if (random_evasion)          //random  evasion
    {
          int N = 1+rand() % 4;
          if (N==1)
          {
          evasion_theta=0;                  
          }
            else    if (N==2)
          {
          evasion_theta=M_PI/2;                  
          }
          else    if (N==3)
          {
          evasion_theta=M_PI/4;                  
          }
        else    if (N==4)
          {
          evasion_theta= -M_PI/4;                  
          }
    }
     
    // std::cout <<"evasion_theta=="<<evasion_theta<<std::endl;
    traj_point.velocity_W.x()=evasion_velocity*cos(evasion_theta);
    traj_point.velocity_W.y()=evasion_velocity*sin(evasion_theta);
    traj_point.position_W.x()+=dt*evasion_velocity*cos(evasion_theta);
    traj_point.position_W.y()+=dt*evasion_velocity*sin(evasion_theta);


    trajectory_msg.header.seq = n_seq;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.points.clear();
    n_seq++;
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(traj_point, &trajectory_point_msg);
    trajectory_msg.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(trajectory_msg);
// 
    ros::Duration(dt).sleep();
    ros::spinOnce();
  }
  
  ros::shutdown();
  return 0;
}
