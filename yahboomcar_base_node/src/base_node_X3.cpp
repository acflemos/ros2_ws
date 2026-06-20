// Nó de odometria para o ROSMaster X3 (rodas mecanum — cinemática omnidirecional).
// Subscreve velocidades brutas do driver de motores e integra-as em posição 2D.
// Publica /odom_raw e opcionalmente o TF odom→base_footprint.
//
// NOTA: o TF odom→base_footprint deve ser publicado por este nó OU pelo robot_localization
// (EKF), nunca pelos dois ao mesmo tempo — configurar pub_odom_tf:=false quando usar EKF.

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <chrono>
#include <functional>

using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node
{
   // Subscrição a velocidades brutas vindas do driver (Mcnamu_driver_X3.py → /vel_raw)
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
   // Publicação de odometria consumida por robot_localization/EKF e Nav2
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

   // Fatores de calibração para compensar deslizamento das rodas (default 1.0)
   double linear_scale_x_ = 0.0;
   double linear_scale_y_ = 0.0;

   double vel_dt_ = 0.0;         // intervalo de tempo entre callbacks (s)
   double x_pos_ = 0.0;          // posição integrada X (m)
   double y_pos_ = 0.0;          // posição integrada Y (m)
   double heading_ = 0.0;        // orientação integrada (rad, yaw)

   double linear_velocity_x_ = 0.0;
   double linear_velocity_y_ = 0.0;   // lateral — válido para mecanum
   double angular_velocity_z_ = 0.0;

   double wheelbase_ = 0.25;     // distância entre eixos (não usado na cinemática X3)
   bool pub_odom_tf_ = false;

   rclcpp::Time last_vel_time_;

public:
   OdomPublisher()
   : Node("base_node")
   {
      this->declare_parameter<double>("wheelbase", 0.25);
      this->declare_parameter<std::string>("odom_frame", "odom");
      this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
      this->declare_parameter<double>("linear_scale_x", 1.0);
      this->declare_parameter<double>("linear_scale_y", 1.0);
      this->declare_parameter<bool>("pub_odom_tf", true);

      this->get_parameter<double>("linear_scale_x", linear_scale_x_);
      this->get_parameter<double>("linear_scale_y", linear_scale_y_);
      this->get_parameter<double>("wheelbase", wheelbase_);
      this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Tópico relativo (sem "/") — resolve para o namespace do nó
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
         "vel_raw", 50, std::bind(&OdomPublisher::handle_vel, this, _1));
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 50);
   }

private:
   void handle_vel(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
   {
      rclcpp::Time current_time = rclcpp::Clock().now();

      linear_velocity_x_ = msg->linear.x * linear_scale_x_;
      linear_velocity_y_ = msg->linear.y * linear_scale_y_;  // movimento lateral mecanum
      angular_velocity_z_ = msg->angular.z;

      vel_dt_ = (current_time - last_vel_time_).seconds();
      last_vel_time_ = current_time;

      // Cinemática omnidirecional: rodar o vetor de velocidade pelo heading atual
      double delta_heading = angular_velocity_z_ * vel_dt_;
      double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_;
      double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_;

      x_pos_ += delta_x;
      y_pos_ += delta_y;
      heading_ += delta_heading;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, heading_);
      geometry_msgs::msg::Quaternion odom_quat;
      odom_quat.x = q.x();
      odom_quat.y = q.y();
      odom_quat.z = q.z();
      odom_quat.w = q.w();

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";

      odom.pose.pose.position.x = x_pos_;
      odom.pose.pose.position.y = y_pos_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;
      odom.pose.covariance[0] = 0.001;
      odom.pose.covariance[7] = 0.001;
      odom.pose.covariance[35] = 0.001;

      odom.twist.twist.linear.x = linear_velocity_x_;
      odom.twist.twist.linear.y = linear_velocity_y_;  // CORRIGIDO: publicar vy real (mecanum)
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      odom.twist.twist.angular.z = angular_velocity_z_;
      odom.twist.covariance[0] = 0.0001;
      odom.twist.covariance[7] = 0.0001;
      odom.twist.covariance[35] = 0.0001;

      odom_publisher_->publish(odom);

      if (pub_odom_tf_)
      {
         geometry_msgs::msg::TransformStamped t;
         t.header.stamp = this->get_clock()->now();
         t.header.frame_id = "odom";
         t.child_frame_id = "base_footprint";
         t.transform.translation.x = x_pos_;
         t.transform.translation.y = y_pos_;
         t.transform.translation.z = 0.0;
         t.transform.rotation.x = q.x();
         t.transform.rotation.y = q.y();
         t.transform.rotation.z = q.z();
         t.transform.rotation.w = q.w();
         tf_broadcaster_->sendTransform(t);
      }
   }
};


int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<OdomPublisher>());
   rclcpp::shutdown();
   return 0;
}
