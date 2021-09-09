#include "main.h"
#include "mobilerobot.h"

stellaN1_node::stellaN1_node():Node("MDC24D100node")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,std::bind(&stellaN1_node::command_velocity_callback,this,std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    
  update_timer_ = this->create_wall_timer(10ms, std::bind(&stellaN1_node::update_callback, this));
}
stellaN1_node::~stellaN1_node()
{

}

void stellaN1_node::update_callback()
{
  MDC24D_rpm_to_pulse(id,left_rpm,right_rpm,encoder);
  rclcpp::Time time_now = this->now();
  // odom
  update_odometry();
  odom.header.stamp = time_now;
  odom_pub_->publish(odom);
  
}

void stellaN1_node::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
    
  goal_linear_velocity_ = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;
  
  calculate_wheel_vel(cmd_vel_msg->linear.x, cmd_vel_msg->angular.z, &left_rpm, &right_rpm);

}

bool stellaN1_node::update_odometry()
{
  /*
  double wheel_l, wheel_r;  // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];
  double step_time = 0;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / wheel_radius_;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / wheel_radius_;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * step_time;
  wheel_r = w[RIGHT] * step_time;

  if (isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / step_time;     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / step_time;  // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pose_[2]);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];
  */
  
  delta_left  = (encoder[0] - left_encoder_prev) *-1;
  delta_right = (encoder[1] - right_encoder_prev);
  
  if (abs(delta_left) < 12000 && abs(delta_right) < 12000)
  {
      delta_s = (delta_left + delta_right) / 2.0 / pulse_per_distance;
      delta_th = ((delta_right - delta_left) / wheel_to_wheel_d / pulse_per_distance);
      delta_x = (delta_s * cos(th + delta_th / 2.0));
      delta_y = (delta_s * sin(th + delta_th / 2.0));
  }
  
  x -= delta_x;
  y -= delta_y;
  th += delta_th;
  
  //geometry_msgs::msg::Quaternion Quaternion = tf2::createQuaternionMsgFromYaw(th);
  
  //transform.setOrigin( tf2::Vector3(x, y,0));
  //transform.setRotation(tf2::Quaternion(Quaternion.x,Quaternion.y,Quaternion.z,Quaternion.w));
  
  tf2::Quaternion Quaternion;
  Quaternion.setRPY(0,0,th);
  
  odom.pose.pose.orientation.x = Quaternion.x();
  odom.pose.pose.orientation.y = Quaternion.y();
  odom.pose.pose.orientation.z = Quaternion.z();
  odom.pose.pose.orientation.w = Quaternion.w();
  /*
  geometry_msgs::msg::TransformStamped t;
  rclcpp::Time time_now = this->now();
  
  t.header.stamp = time_now;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  
  t.transform.rotation.x= Quaternion.x();
  t.transform.rotation.y= Quaternion.y();
  t.transform.rotation.z= Quaternion.z();
  t.transform.rotation.w= Quaternion.w();
  
  
  odom_broadcaster->sendTransform(t);
  */

  nav_msgs::msg::Odometry odom;

  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x = goal_linear_velocity_;
  odom.twist.twist.angular.z = goal_angular_velocity_;
  
  left_encoder_prev = encoder[0];
  right_encoder_prev = encoder[1];
  

  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  MW_SerialOpen("/dev/MW",115200);
  sleep(0.2);
  printf("Connect Sucess\n");
  
  rclcpp::spin(std::make_shared<stellaN1_node>());
  rclcpp::shutdown();
  return 0;
}
