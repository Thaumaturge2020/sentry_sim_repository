#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/float64.hpp>
#include "task_msg/msg/float64_multi_array.hpp"
#include "task_msg/srv/point2d_move.hpp"
#include "task_msg/srv/path_plan.hpp"

//最离谱的方式：探头式避障，应该行吧有一说一

namespace localPlanningCpp{
    const double target_len = 0.03;
    const double max_velo = 3;
    const double max_ang_velo = 6;
    const double pi = 3.1415926535;
    const double lowest_dis = 0.1;
    const double min_ang_db = 0.01;
    const double max_ang_db = 0.3;
    class LocalPlanner:public rclcpp::Node{
        private:
        double sub_ang_velo;
        Eigen::Vector2d sub_pos,sub_velo;
        double pub_velo,pub_ang_velo;
        public:
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_velo;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_ang_velo;
        rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_endpoint2d;
        rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_velo;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_ang_velo;
        rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_pos;
        rclcpp::Client<task_msg::srv::Point2dMove>::SharedPtr client_movepoint;
        rclcpp::Service<task_msg::srv::PathPlan>::SharedPtr server_nav_local;

        std::shared_ptr<rclcpp::Node> node_local_server;

        std::shared_ptr<rclcpp::Node> node_movepoint;
        rclcpp::TimerBase::SharedPtr timer_;
        Eigen::Vector2d end_point_this;
        int end_point_flag,sub_velo_flag,sub_ang_velo_flag,sub_pos_flag;
        explicit LocalPlanner(const rclcpp::NodeOptions & options)
            : Node("local_planner", options){
            RCLCPP_INFO(this->get_logger(),"client starting...");
            using namespace std::placeholders;
            publisher_velo = this->create_publisher<std_msgs::msg::Float64>("local_velocity", 10);
            publisher_ang_velo = this->create_publisher<std_msgs::msg::Float64>("local_angular_velocity", 10);
            // subscriber_endpoint2d = this->create_subscription<task_msg::msg::Float64MultiArray>("end_point_local_planner", 10, std::bind(&LocalPlanner::end_point_callback, this, _1));
            // subscriber_velo = this->create_subscription<task_msg::msg::Float64MultiArray>("meter_velocity",10,std::bind(&LocalPlanner::meter_velo_callback, this, _1));
            // subscriber_ang_velo = this->create_subscription<std_msgs::msg::Float64>("meter_angular_velocity",10,std::bind(&LocalPlanner::meter_ang_velo_callback, this, _1));
            // subscriber_pos = this->create_subscription<task_msg::msg::Float64MultiArray>("meter_position",10,std::bind(&LocalPlanner::meter_position_callback, this, _1));
            node_movepoint = rclcpp::Node::make_shared("point_mover_client");
            node_local_server = rclcpp::Node::make_shared("nav_local_server");
            server_nav_local = node_local_server->create_service<task_msg::srv::PathPlan>("local_navigation",std::bind(&LocalPlanner::local_direction_service,this,_1,_2));
            client_movepoint = node_movepoint->create_client<task_msg::srv::Point2dMove>("point_mover");
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&LocalPlanner::timer_callback, this));
            end_point_flag = sub_ang_velo_flag = sub_velo_flag = sub_pos_flag = 0;
            std::thread{std::bind(&LocalPlanner::local_navigation_start,this)}.detach();
        }
        void local_navigation_start(){rclcpp::spin(node_local_server);rclcpp::shutdown();}
        void end_point_callback(task_msg::msg::Float64MultiArray array){
            auto pos2d = array.data;
            end_point_this = Eigen::Map<Eigen::Vector2d>(pos2d.data());
            end_point_flag = 1;
            return;
        }
        void meter_velo_callback(task_msg::msg::Float64MultiArray msg){sub_velo = Eigen::Map<Eigen::Vector2d>(msg.data.data());sub_velo_flag = 1;}
        void meter_ang_velo_callback(std_msgs::msg::Float64 msg){sub_ang_velo = msg.data;sub_ang_velo_flag = 1;}
        void meter_position_callback(task_msg::msg::Float64MultiArray msg){sub_pos = Eigen::Map<Eigen::Vector2d>(msg.data.data());sub_pos_flag = 1;}
        Eigen::Vector2d get_velo_from_platform(){return sub_velo;}
        double get_ang_velo_from_platform(){return sub_ang_velo;}
        Eigen::Vector2d get_pos_from_platform(){return sub_pos;}
        void timer_callback(){
            if(end_point_flag + sub_ang_velo_flag + sub_pos_flag + sub_velo_flag < 4) return;
            local_direction(end_point_this);
            return;
        }
        void publish_ang_velo(double now_ang_velo){
            pub_ang_velo = now_ang_velo;
            // std_msgs::msg::Float64 now_ang_velo_msg;
            // now_ang_velo_msg.data = now_ang_velo;
            // publisher_ang_velo->publish(now_ang_velo_msg);
            return;
        }
        void publish_velo(double now_velo){
            pub_velo = now_velo;
            // std_msgs::msg::Float64 now_velo_msg;
            // now_velo_msg.data = now_velo;
            // publisher_velo->publish(now_velo_msg);
            return;
        }

        Eigen::Vector2d get_from_platform_autoshaped_point(Eigen::Vector2d target_point){
            auto request = std::make_shared<task_msg::srv::Point2dMove::Request>();
            std::vector<double> target_point_array;
            target_point_array.push_back(target_point[0]);
            target_point_array.push_back(target_point[1]);
            request->ori.data = target_point_array;
            while (!client_movepoint->wait_for_service(std::chrono::milliseconds(1000)));
            auto result = client_movepoint->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_movepoint,result) == rclcpp::FutureReturnCode::SUCCESS)
            target_point_array = result.get()->tar.data;
            return Eigen::Map<Eigen::Vector2d>(target_point_array.data());
        }

        
        
        void local_direction(Eigen::Vector2d end_point){

            static const double pi = 3.14159265358979323;

            auto start_point = get_pos_from_platform();
            auto velo = get_velo_from_platform();
            Eigen::Vector2d target_vec = end_point-start_point;
            target_vec = target_vec*target_len/target_vec.norm();
            Eigen::Vector2d target_point = target_vec + start_point;
            // target_point = get_from_platform_autoshaped_point(target_point);
            target_vec = target_point - start_point;

            double  ret = atan2(target_vec[1],target_vec[0]) - velo[0],
                    now_ang_velo = 0,
                    now_velo = 0;

            while(ret < -pi) ret += 2*pi;
            while(ret > pi)  ret -= 2*pi;

            // RCLCPP_INFO(this->get_logger(),"angle:%lf velo_ang:%lf velo:%lf pos:[%lf,%lf]",ret,velo[0],velo[1],start_point[0],start_point[1]);

            if((start_point-end_point).norm() < lowest_dis){
                publish_ang_velo(now_ang_velo);
                publish_velo(now_velo);
                end_point_flag = 0;
                return;
            }
            
            if(ret > min_ang_db)
            now_ang_velo = max_ang_velo;

            if(ret < -min_ang_db)
            now_ang_velo = -max_ang_velo;

            if(fabs(ret) < max_ang_db)
            now_velo = max_velo;

            // RCLCPP_INFO(this->get_logger(),"publish:now_ang_velo:%lf,now_velo:%lf",now_ang_velo,now_velo);

            publish_ang_velo(now_ang_velo);
            publish_velo(now_velo);
            return;
        }

        void local_direction_service(const std::shared_ptr<task_msg::srv::PathPlan::Request> request,std::shared_ptr<task_msg::srv::PathPlan::Response> response){
            sub_velo = Eigen::Map<Eigen::Vector2d>(request->old_velo.data.data());
            sub_pos = Eigen::Map<Eigen::Vector2d>(request->start_point.data.data());
            local_direction(Eigen::Map<Eigen::Vector2d>(request->end_point.data.data()));
            response->velo.data = std::vector<double>{pub_ang_velo,pub_velo};
            return;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(localPlanningCpp::LocalPlanner)