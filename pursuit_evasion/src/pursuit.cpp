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
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#define SQ(x) ((x)*(x))

nav_msgs::Odometry evasion_odom;
nav_msgs::Odometry self_odom;
nav_msgs::Odometry  pursuits_odom[3]={};
  int team_num=3;       // 3 pursuers.
  float pursuit_velocity=0.8;               // evasion velocity  m/s
  float evasion_velocity=0.5;
ros::Publisher Apollonius_vis_pub_;
int id=0;

void evasion_callback( const nav_msgs::Odometry &evasion_odom_msgs)
{
  // std::cout <<"==evasion_callback==="<<std::endl;
  evasion_odom=evasion_odom_msgs;
}

void selfodom_callback( const nav_msgs::Odometry &self_odom_msgs)
{
  // std::cout <<"==evasion_callback==="<<std::endl;
  self_odom=self_odom_msgs;

}

void pursuit1_odom_callback( const nav_msgs::Odometry &odom_msgs)
{
  pursuits_odom[0]=odom_msgs;
}
void pursuit2_odom_callback( const nav_msgs::Odometry &odom_msgs)
{
  pursuits_odom[1]=odom_msgs;
}
void pursuit3_odom_callback( const nav_msgs::Odometry &odom_msgs)
{
  pursuits_odom[2]=odom_msgs;
}

void publishVisApollonius(float x, float y , float r)
{

  visualization_msgs::Marker   p;
   std::cout<<"x,y===="<<x<<"  ,"<<y<<std::endl;

  p.header.stamp = ros::Time::now();
  p.header.seq = id;
  p.header.frame_id = "world";
  p.id =id ;
  id++;
  p.ns = "Apollonius";
  p.type = visualization_msgs::Marker::CYLINDER;
  p.action = visualization_msgs::Marker::ADD;
  
  p.pose.position.x=x;
  p.pose.position.y=y;
  p.pose.position.z=0;

  p.scale.x = 2*r;
  p.scale.y = 2*r;
  p.scale.z = 0.1;

  p.color.r = 0.0;
  p.color.g =id*10 / 255.0;
  p.color.b = id*50 / 255.0;
  p.color.a = 0.3;
  p.lifetime = ros::Duration(0);

  Apollonius_vis_pub_.publish(p);


}
void publishVisInterPoint(float x, float y )
{

  visualization_msgs::Marker   p;

  p.header.stamp = ros::Time::now();
  p.header.seq = id;
  p.header.frame_id = "world";
  p.id =id ;
  id++;
  p.ns = "Apollonius_inter";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  
  p.pose.position.x=x;
  p.pose.position.y=y;
  p.pose.position.z=0;

  p.scale.x = 0.4;
  p.scale.y =0.4;
  p.scale.z = 0.1;

  p.color.r =190/255;
  p.color.g =0/255;
  p.color.b =0/255.0;
  p.color.a = 1;
  p.lifetime = ros::Duration(0);

  Apollonius_vis_pub_.publish(p);


}

bool ApolloniusAllocation(int pursuit_num )
{
    // if all intersection outside one Apollonius circle, the uav become redundant 
  // std::cout<<"pursuit_num===== "<<pursuit_num<<std::endl;
  float Ax[3];
  float Ay[3];
  float Ar[3];
  float rou=evasion_velocity/pursuit_velocity;
  // std::cout <<"rou==="<<rou<<std::endl;
  // std::cout<<"evasion_odom===== "<<evasion_odom.pose.pose.position<<std::endl;

  for (int i =0;i <=2;i++)
    {        

            // std::cout<<"pursuits_odom===== "<<pursuits_odom[i].pose.pose.position<<std::endl;
                    Ax[i]=(evasion_odom.pose.pose.position.x  - rou*rou*pursuits_odom[i].pose.pose.position.x)/(1-rou*rou);
                    Ay[i]=(evasion_odom.pose.pose.position.y  - rou*rou*pursuits_odom[i].pose.pose.position.y)/(1-rou*rou);
                    Ar[i]=(  rou / (1-rou*rou) )*sqrt(
                    pow(pursuits_odom[i].pose.pose.position.x-evasion_odom.pose.pose.position.x,2)+
                    pow(pursuits_odom[i].pose.pose.position.y-evasion_odom.pose.pose.position.y,2) );
    }

  // std::cout <<"A====="<<Ax[0]<<" , "<<Ay[0]<<"        "<<Ax[1]<<" , "<<Ay[1]<<"        "<<Ax[2]<<" , "<<Ay[2]<< std::endl;
  // std::cout <<"Ar====="<<Ar[0]<<"   "<<Ar[1]<<"    "<<Ar[2]<<"    "<<std::endl;
  std:: vector<float> intersectionX;
  std:: vector<float> intersectionY;

  for (int i=0; i<3;i++)
  {
  publishVisApollonius(Ax[i],Ay[i],Ar[i]);
  }

  for (int i =0;i <=2;i++)
               {

                            if( i!=pursuit_num  &&
                            sqrt(pow(Ax[pursuit_num]-Ax[i],2)+pow(Ay[pursuit_num]-Ay[i],2))<Ar[pursuit_num]+Ar[i]
                            )
                            {
                                // std::cout <<"Ay[pursuit_num]"<<Ay[pursuit_num]<<"  , "<<"Ay[i]=="<<Ay[i]<<std::endl;

                                float k= -(Ax[i] - Ax[pursuit_num])/(Ay[i]-Ay[pursuit_num]);
                                float b=
                                (       SQ(Ax[i])-SQ(Ax[pursuit_num]) +   SQ(Ay[i]) - SQ(Ay[pursuit_num]) -   (SQ(Ar[i])-SQ(Ar[pursuit_num]) ) )
                                  /   (2*Ay[i]-2*Ay[pursuit_num]);
                                
                                float x0=(Ax[pursuit_num]+k*Ay[pursuit_num]-b*k)/(1+SQ(k));
                                float y0=k*x0+b;

                                float OA=  (k*Ax[pursuit_num]-Ay[pursuit_num]+b)/sqrt(1+SQ(k));
                                float OB=sqrt(SQ(Ar[pursuit_num]) - SQ(OA));

                                float  x_intersection1=x0+OB*(1/sqrt(1+SQ(k)));
                                float  y_intersection1=y0 +OB*(k/sqrt(1+SQ(k)));
                                float  x_intersection2=x0- OB*(1/sqrt(1+SQ(k)));
                                float  y_intersection2=y0 - OB*(k/sqrt(1+SQ(k)));

                                intersectionX.push_back(x_intersection1);
                                intersectionY.push_back(y_intersection1);
                                intersectionX.push_back(x_intersection2);
                                intersectionY.push_back(y_intersection2);                              
                    }
               }

  int InterSize=intersectionX.size();

  for(int i =0;i <InterSize;i++)
  {
    // std::cout<<"intersectionXY===="<<intersectionX[i]<<"  ,"<<intersectionY[i]<<std::endl;
    publishVisInterPoint(intersectionX[i],intersectionY[i]);
  }


  for ( int i =0; i <InterSize;i++)
  {
            int count=0;
            for (int j =0; j <3; j++)
            {       
                    float d=sqrt( (intersectionX[i]-Ax[j])*(intersectionX[i]-Ax[j])+  (intersectionY[i]-Ay[j])*(intersectionY[i]-Ay[j]) );
                    if ((d<=Ar[j]+0.3) && j!=pursuit_num)
                    {
                        count++;
                    }
              }
              
            if(count==2)
            {
              return true;  // there is a  intersection in the all Apollonius circles , we can then get a collisonfree line ,so  return true,
            }
  }
    // if  process go  here, it means that there isnt a intersection in the all Apollonius circles. so return false;
  if (InterSize>0)
  {
 std::cout <<"InterSize>0"<<std::endl;

  return false;
  }
  else{
    for (int i=0;i <3;i++)
    {
      if (Ar[team_num]<Ar[i])
      {
        return true;
      }
      else{
          std::cout <<"InterSize==0,,, big circle"<<std::endl;
        return false;
      }
    }

  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started hovering example.");
  Apollonius_vis_pub_= nh.advertise<visualization_msgs::Marker>("Apollonius_vis", 10);
  ros::Subscriber  evasion_sub=  nh.subscribe("/hummingbird1/ground_truth/odometry",10,evasion_callback);
  ros::Subscriber  selfodom_sub=  nh.subscribe("ground_truth/odometry",10,selfodom_callback);

  ros::Subscriber  pursuit1_odom_sub=  nh.subscribe("/firefly2/ground_truth/odometry",10,pursuit1_odom_callback);
  ros::Subscriber  pursuit2_odom_sub=  nh.subscribe("/firefly3/ground_truth/odometry",10,pursuit2_odom_callback);
  ros::Subscriber  pursuit3_odom_sub=  nh.subscribe("/firefly4/ground_truth/odometry",10,pursuit3_odom_callback);
  // int pursuit_num= int ( nh.getNamespace()-"firefly")-48;

  int pursuit_num= int (nh.getNamespace()[8])-48-2;


  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(2.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 2.0);
  double desired_yaw = 0.0;
  float evasion_theta=0;
  bool Apollonius =false;
  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);
  nh_private.param("evasion_theta", evasion_theta, evasion_theta);
  nh_private.param("Apollonius", Apollonius, Apollonius);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
    
  ros::Duration(2.0).sleep();



  float pursuit_theta=0;               // evasion velocity  m/s
  float phi=0;

  // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
  //          nh.getNamespace().c_str(), desired_position.x(),
  //          desired_position.y(), desired_position.z());

  trajectory_pub.publish(trajectory_msg);
  ros::Duration(2.0).sleep();

  mav_msgs::EigenTrajectoryPoint traj_point;

  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  traj_point.position_W.x()=desired_position.x();
  traj_point.position_W.y()=desired_position.y();
  traj_point.position_W.z()=desired_position.z();
  
  float dt=0.05;
  static int n_seq = 0;

  ros::spinOnce();

bool redundant_flag=false;
     if (Apollonius)
    {
     if ( ! ApolloniusAllocation(pursuit_num))   

       {
        redundant_flag=true;
      }
    }

  while ( ros::ok())
  {


  if (redundant_flag)
  {
    continue;
  }


    float distance_x=evasion_odom.pose.pose.position.x-self_odom.pose.pose.position.x;
    float distance_y=evasion_odom.pose.pose.position.y-self_odom.pose.pose.position.y;

                   

      float evasion_theta=atan(evasion_odom.twist.twist.linear.y/evasion_odom.twist.twist.linear.x);       //            get  evasion_theta
      if(evasion_odom.twist.twist.linear.y<0 &&evasion_odom.twist.twist.linear.x<0)
      {
      evasion_theta=evasion_theta+M_PI;
      }
      else if (evasion_odom.twist.twist.linear.y>0 &&evasion_odom.twist.twist.linear.x<0)
      {
      evasion_theta=evasion_theta+M_PI;
      }


    if (distance_y*distance_y+distance_x*distance_x<0.1)                                     //           captured
    {       
    traj_point.velocity_W.x()=evasion_velocity*cos(evasion_theta);             // motion    command  calculation
    traj_point.velocity_W.y()=evasion_velocity*sin(evasion_theta);
    traj_point.position_W.x()+=dt*evasion_velocity*cos(evasion_theta);
    traj_point.position_W.y()+=dt*evasion_velocity*sin(evasion_theta);
    }
    else{
                    phi=atan(distance_y/distance_x);
                    //            get  phi
                    if(distance_y<0 && distance_x<0)
                    {
                      phi=phi+M_PI;
                    }
                    else   if(distance_y>0 && distance_x<0)

                    {
                    phi=phi+M_PI;
                    }

                    pursuit_theta=phi+asin( (evasion_velocity/pursuit_velocity)
                                                                                        * sin(evasion_theta-phi) );

                    traj_point.velocity_W.x()=pursuit_velocity*cos(pursuit_theta);                                          // motion    command  calculation
                    traj_point.velocity_W.y()=pursuit_velocity*sin(pursuit_theta);
                    traj_point.position_W.x()+=dt*pursuit_velocity*cos(pursuit_theta);
                    traj_point.position_W.y()+=dt*pursuit_velocity*sin(pursuit_theta);
    }

    trajectory_msg.header.seq = n_seq;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.points.clear();
    n_seq++;
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(traj_point, &trajectory_point_msg);
    trajectory_msg.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(trajectory_msg);


    ros::Duration(dt).sleep();
    ros::spinOnce();
  }
  
  ros::shutdown();
  return 0;
}
