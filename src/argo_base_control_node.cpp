#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class ArgoBaseController
{
public:
  ArgoBaseController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher argo_twist_pub_;
  ros::Publisher argo_twist_pub_R;
  ros::Subscriber argo_joy_sub_;
  
};


ArgoBaseController::ArgoBaseController():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  argo_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("argo_base/cmd_vel", 1);
  argo_twist_pub_R = nh_.advertise<geometry_msgs::Twist>("roboteq_driver/argo_base/cmd_vel", 1);
  argo_joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArgoBaseController::joyCallback, this);

}

void ArgoBaseController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  argo_twist_pub_.publish(twist);
  argo_twist_pub_R.publish(twist);
   
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "argo_base_controller");
  ArgoBaseController ArgoBaseController;

  ros::spin();
}
