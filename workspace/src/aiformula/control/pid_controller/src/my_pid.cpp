#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cmath>
#include <vector>

class AutoDriveController : public rclcpp::Node
{
public:
    AutoDriveController()
    : Node("auto_drive_controller"),
      p1(1.0), p2(2.0), p3(3.0),
      Kp0(1.0), Ki0(1.0), Kd0(1.0),
      Kp1(2.0), Ki1(2.0), Kd1(2.0),
      v_error_history(3, 0.0), omega_error_history(3, 0.0)
    {
        target_pose_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "target_pose", 10, std::bind(&AutoDriveController::targetPoseCallback, this, std::placeholders::_1));

        current_pose_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "current_pose", 10, std::bind(&AutoDriveController::currentPoseCallback, this, std::placeholders::_1));

        current_velocity_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "current_velocity", 10, std::bind(&AutoDriveController::currentVelocityCallback, this, std::placeholders::_1));

        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    double p1, p2, p3;
    double Kp0, Ki0, Kd0; // PID gains for linear velocity
    double Kp1, Ki1, Kd1; // PID gains for angular velocity
    double ve, omega_e;

    std::vector<double> v_error_history;
    std::vector<double> omega_error_history;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr current_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_velocity_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    geometry_msgs::msg::Pose2D target_pose;
    geometry_msgs::msg::Pose2D current_pose;
    geometry_msgs::msg::Twist current_velocity;

    void targetPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        target_pose = *msg;
    }

    void currentPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        current_pose = *msg;
        computeControl();
    }

    void currentVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_velocity = *msg;
        v_error_history.insert(v_error_history.begin(), ve);
        omega_error_history.insert(omega_error_history.begin(), omega_e);
        if (v_error_history.size() > 3) v_error_history.pop_back();
        if (omega_error_history.size() > 3) omega_error_history.pop_back();

    }

    void computeControl() {
        double xe, ye, theta_e, re, vr, omega_r;

        xe = target_pose.x - current_pose.x;
        ye = target_pose.y - current_pose.y;
        theta_e = target_pose.theta - current_pose.theta;

        re = sqrt(xe * xe + ye * ye);
        vr = p1 * re * cos(theta_e);
        omega_r = p2 * theta_e + p1 * ((cos(theta_e) * sin(theta_e)) / theta_e) * (theta_e + p3 * target_pose.theta);

        ve = vr - current_velocity.linear.x;
        omega_e = omega_r - current_velocity.angular.z;

        v_error_history.insert(v_error_history.begin(), ve);
        omega_error_history.insert(omega_error_history.begin(), omega_e);
        if (v_error_history.size() > 3) v_error_history.pop_back();
        if (omega_error_history.size() > 3) omega_error_history.pop_back();


        double delta_v = Kp0 * ve + Ki0 * v_error_history[1] + Kd0 * v_error_history[2];
        double delta_omega = Kp1 * omega_e + Ki1 * omega_error_history[1] + Kd1 * omega_error_history[2];

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = current_velocity.linear.x + delta_v;
        cmd_vel.angular.z = current_velocity.angular.z + delta_omega;

        cmd_vel_pub->publish(cmd_vel);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoDriveController>());
    rclcpp::shutdown();
    return 0;
}
