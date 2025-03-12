#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>

class TurtleController : public rclcpp::Node {
public:
    TurtleController() : Node("turtle_controller") {
        RCLCPP_INFO(this->get_logger(), "TurtleController node started.");
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleController::pose_callback, this, std::placeholders::_1)
        );
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/goal", 10,
            std::bind(&TurtleController::goal_callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
    geometry_msgs::msg::Point goal_;
    bool goal_received_ = false;
    const double linear_velocity = 2.0; // Lineer velocity
    const double angular_gain = 2.0;   // Angular gain
    const double goal_threshold = 0.1; // Distance threshold for goal

    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal received: [%.2f, %.2f]", goal_.x, goal_.y);
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        if (!goal_received_) return;

        double dx = goal_.x - msg->x;
        double dy = goal_.y - msg->y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < goal_threshold) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!\n");
            goal_received_ = false;
            stop_turtle();
            return;
        }

        double goal_theta = std::atan2(dy, dx);
        double theta_error = goal_theta - msg->theta;

        // Açıyı -PI ile PI arasına normalleştir
        while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
        while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::min(linear_velocity, distance); // Lineer control
        cmd.angular.z = angular_gain * theta_error;           // Angular control

        // Pencere sınırlarına yaklaşmayı kontrol et (örneğin, turtlesim için güvenli alan [1.0, 10.0])
        const double safe_margin = 0.0;
        if ((msg->x <= safe_margin && std::cos(msg->theta) < 0) ||
            (msg->x >= 11.0 && std::cos(msg->theta) > 0) ||
            (msg->y <= safe_margin && std::sin(msg->theta) < 0) ||
            (msg->y >= 11.0 && std::sin(msg->theta) > 0)) {
            RCLCPP_WARN(this->get_logger(), "Near boundary, stopping forward motion.");
            cmd.linear.x = 0.0;
        }

        cmd_vel_pub_->publish(cmd);
    } 

    void stop_turtle() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}
