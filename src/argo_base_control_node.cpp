/**
Software License Agreement (BSD)

\file      driver.cpp
\authors   Bismaya Sahoo <bsahoo@uwaterloo.ca>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <rosserial_arduino/Adc.h>

//Brake Limit ADC Values HARD
#define HARD_HIGH_0 300
#define HARD_LOW_0 130

#define HARD_HIGH_1 275
#define HARD_LOW_1 140


class ArgoBaseController
{
public:
  ArgoBaseController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void brakeCallback(const rosserial_arduino::Adc::ConstPtr& brakeInfo);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;

  ros::Publisher argo_twist_pub_;
  ros::Publisher argo_twist_pub_R;
  ros::Subscriber argo_joy_sub_;
  ros::Subscriber argo_brake_sub;
  
  int leftBrakePos, rightBrakePos;
  
};


ArgoBaseController::ArgoBaseController():
  linear_(1),
  angular_(2)
{

  leftBrakePos=140;//Min. Will soon be upadated as soon as feedback is available
  rightBrakePos=140;//Min. Will soon be updated as soon as feedback is available
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  argo_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("argo_base/cmd_vel", 1);
  argo_twist_pub_R = nh_.advertise<geometry_msgs::Twist>("roboteq_driver/argo_base/cmd_vel", 1);
  argo_joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArgoBaseController::joyCallback, this);
  argo_brake_sub = nh_.subscribe<rosserial_arduino::Adc>("roboteq_driver/brakeInfo",10, &ArgoBaseController::brakeCallback, this);

}

void ArgoBaseController::brakeCallback(const rosserial_arduino::Adc::ConstPtr& brakeInfo)
{
  leftBrakePos=brakeInfo->adc1;
  rightBrakePos=brakeInfo->adc0;
  //hacky. Publish this info as Twist linear y and z

  ROS_INFO("%d,%d",leftBrakePos,rightBrakePos);
}

void ArgoBaseController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
 // twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  
  twist.linear.y = leftBrakePos;
  twist.linear.z = rightBrakePos;  
  //HAVE TO KEEP PROVISIONS FOR IF FEEDBACK IS ENABLED
  //If either crossess limit dont force it to go either way
  if (leftBrakePos<HARD_LOW_1 || leftBrakePos>HARD_HIGH_1)
    twist.angular.x=0;
  else if (rightBrakePos<HARD_LOW_0 ||rightBrakePos>HARD_HIGH_0)
    twist.angular.y=0;
  //0=No action,1=FWD,-1=RVS
  if (joy->buttons[4]==1)
    twist.angular.x = 1;
  else
    twist.angular.x =-1;
  if (joy->buttons[5]==1)
    twist.angular.y = 1;
  else
    twist.angular.y = -1;

  argo_twist_pub_.publish(twist);
  argo_twist_pub_R.publish(twist);
   
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "argo_base_controller");
  ArgoBaseController ArgoBaseController;

  ros::spin();
}
