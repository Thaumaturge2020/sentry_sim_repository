#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <toml.hpp>
#include <utility>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include "task_msg/msg/float64_pair_multi_array.hpp"
#include "task_msg/msg/float64_multi_array.hpp"
#include "task_msg/srv/point2d_move.hpp"
#include "task_msg/srv/nav_find.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace behaviourDecision{
    class BehaviourTree:public rclcpp::Node{
        public:
        rclcpp::Publisher<task_msg::msg::Float64MultiArray>::SharedPtr  publisher_global_plan_pos;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr  publisher_shooter;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  publisher_avail_angle;
        rclcpp::TimerBase::SharedPtr timer_;
        explicit BehaviourTree(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("behaviour_tree", options){
                timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&BehaviourTree::behaviour_publish, this));
                publisher_global_plan_pos = this->create_publisher<task_msg::msg::Float64MultiArray>("end_point_global_planner",10);
                publisher_avail_angle = this->create_publisher<std_msgs::msg::Float64>("decision_shooting_angle",10);
                publisher_shooter = this->create_publisher<std_msgs::msg::Int64>("decision_shooting_able",10);
            }
        void behaviour_publish(){
            const double pi = 3.14159265358979323;
            std::vector<double> vec;
            static double CNT = 0;
            while(CNT > 2*pi) CNT -= 2*pi;
            double angle = ++CNT;
            std_msgs::msg::Float64 pub_angle;
            pub_angle.data = angle;
            vec.push_back(1.2);
            vec.push_back(3.16);
            task_msg::msg::Float64MultiArray pub_msg;
            pub_msg.data = vec;
            publisher_global_plan_pos->publish(pub_msg);
            publisher_avail_angle->publish(pub_angle);
            std_msgs::msg::Int64 able_msg;
            able_msg.data = 1;
            publisher_shooter->publish(able_msg);
            return;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(behaviourDecision::BehaviourTree)