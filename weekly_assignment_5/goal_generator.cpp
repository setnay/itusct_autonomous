#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <turtlesim/msg/pose.hpp>
#include <random>
#include <cmath>

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher"), goal_reached_(false), initial_pose_received_(false) {
        goal_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/goal", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&GoalPublisher::pose_callback, this, std::placeholders::_1)
        );
        // İlk hedefi hemen oluşturmayın; pose_callback'da initial pose alındığında oluşturun.
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    geometry_msgs::msg::Point current_goal_;
    bool goal_reached_;
    bool initial_pose_received_;
    turtlesim::msg::Pose initial_pose_;
    // Pencere büyüklüğünün yarısı (turtlesim için yaklaşık 5.5)
    const double min_distance_from_initial_ = 5.5;

    void generate_new_goal() {
        std::random_device rd;
        std::mt19937 gen(rd());
        // Güvenli alan: [1.0, 10.0]
        std::uniform_real_distribution<> dis(1.0, 10.0);

        do {
            current_goal_.x = dis(gen);
            current_goal_.y = dis(gen);
        } while (initial_pose_received_ &&
                 (std::sqrt(std::pow(current_goal_.x - initial_pose_.x, 2) +
                            std::pow(current_goal_.y - initial_pose_.y, 2))
                 < min_distance_from_initial_));

        RCLCPP_INFO(this->get_logger(), "New Goal: [%.2f, %.2f]", current_goal_.x, current_goal_.y);
        goal_pub_->publish(current_goal_);
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        // İlk pose alındıysa ve henüz initial_pose kaydedilmediyse, kaydet ve yeni hedef oluştur.
        if (!initial_pose_received_) {
            initial_pose_ = *msg;
            initial_pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial Pose set: [%.2f, %.2f]", initial_pose_.x, initial_pose_.y);
            generate_new_goal();
            return;
        }

        double dx = msg->x - current_goal_.x;
        double dy = msg->y - current_goal_.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Turtle hedefe ulaştığında yeni hedef yayınla.
        if (distance < 0.1 && !goal_reached_) {
            RCLCPP_INFO(this->get_logger(), "Turtle reached goal, publishing new goal...");
            goal_reached_ = true;
            generate_new_goal();
        } else if (distance >= 0.1) {
            goal_reached_ = false;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
