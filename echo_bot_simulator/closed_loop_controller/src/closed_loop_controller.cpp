#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

class ClosedLoopState
{
public:
  ClosedLoopState(ros::NodeHandle& nh)
  : nh_(nh)
  {
    sub_ = nh_.subscribe("/echo_bot/joint_states", 10, &ClosedLoopState::updateJoints, this);
  }

public:
  std_msgs::Float64 lleg_joint1_;
  std_msgs::Float64 lleg_joint2_;
  std_msgs::Float64 rleg_joint1_;
  std_msgs::Float64 rleg_joint2_;

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  const float A_ = 53.151;
  const float B_ = 100.0;
  const float C_ = 25.0;
  const float D_ = 100.0;
  const float theta_initial_ = 1.0679;
  const float alfa_2_initial_ = calcAlfa2(theta_initial_);
  const float alfa_3_initial_ = calcAlfa3(theta_initial_);
  const float alfa_4_initial_ = calcAlfa4(theta_initial_);

private:
  float calcAlfa2(const float theta);
  float calcAlfa3(const float theta);
  float calcAlfa4(const float theta);
  void updateJoints(const sensor_msgs::JointState::ConstPtr& state);
};

float ClosedLoopState::calcAlfa2(const float theta)
{
  const float L = std::sqrt( pow(A_, 2) + pow(B_, 2) - 2*A_*B_*std::cos(theta) );
  return std::acos( (pow(A_,2)+pow(B_,2)-pow(C_,2)+pow(D_,2)-2*A_*B_*std::cos(theta)) / (2*D_*L) );
}

float ClosedLoopState::calcAlfa3(const float theta)
{
  const float L = std::sqrt( pow(A_, 2) + pow(B_, 2) - 2*A_*B_*std::cos(theta) );
  return std::acos( (B_ - A_*std::cos(theta)) / (L) );
}

float ClosedLoopState::calcAlfa4(const float theta)
{
  const float L = std::sqrt( pow(A_, 2) + pow(B_, 2) - 2*A_*B_*std::cos(theta) );
  return std::acos( (pow(A_,2)+pow(B_,2)+pow(C_,2)-pow(D_,2)-2*A_*B_*std::cos(theta)) / (2*C_*L) );
}

void ClosedLoopState::updateJoints(const sensor_msgs::JointState::ConstPtr& state)
{
  ROS_INFO_ONCE("Subscribed !");
  const float l_theta = theta_initial_ - state->position[2];
  const float r_theta = theta_initial_ - state->position[3];
  const float l_alfa_2_ = calcAlfa2(l_theta);
  const float r_alfa_2_ = calcAlfa2(r_theta);
  const float l_alfa_3_ = calcAlfa3(l_theta);
  const float r_alfa_3_ = calcAlfa3(r_theta);
  const float l_alfa_4_ = calcAlfa4(l_theta);
  const float r_alfa_4_ = calcAlfa4(r_theta);

  lleg_joint1_.data = -1*((l_alfa_3_ - calcAlfa3(theta_initial_)) + (l_alfa_4_ - calcAlfa4(theta_initial_)));
  lleg_joint2_.data = -1*((l_alfa_2_ - l_alfa_4_) - (calcAlfa2(theta_initial_) - calcAlfa4(theta_initial_)));
  rleg_joint1_.data = -1*((r_alfa_3_ - calcAlfa3(theta_initial_)) + (r_alfa_4_ - calcAlfa4(theta_initial_)));
  rleg_joint2_.data = -1*((r_alfa_2_ - r_alfa_4_) - (calcAlfa2(theta_initial_) - calcAlfa4(theta_initial_)));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "closed_loop_controller");
  ros::NodeHandle nh;

  ClosedLoopState closed_loop(nh);

  ros::Publisher lower_lleg_pub       = nh.advertise<std_msgs::Float64>("/echo_bot/lower_left_leg_position_controller/command", 10);
  ros::Publisher lower_rleg_pub       = nh.advertise<std_msgs::Float64>("/echo_bot/lower_right_leg_position_controller/command", 10);
  ros::Publisher upper_side_lleg_pub  = nh.advertise<std_msgs::Float64>("/echo_bot/upper_side_left_leg_position_controller/command", 10);
  ros::Publisher upper_side_rleg_pub  = nh.advertise<std_msgs::Float64>("/echo_bot/upper_side_right_leg_position_controller/command", 10);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    lower_lleg_pub.publish(closed_loop.lleg_joint1_);
    upper_side_lleg_pub.publish(closed_loop.lleg_joint2_);
    lower_rleg_pub.publish(closed_loop.rleg_joint1_);
    upper_side_rleg_pub.publish(closed_loop.rleg_joint2_);

    ros::spinOnce();
  }

  return 0;
}