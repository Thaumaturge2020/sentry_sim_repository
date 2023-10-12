#include "global_planner/global_planner.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <queue>
#include <toml.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <Eigen/Core>

#include <std_msgs/msg/float64.hpp>
#include "task_msg/msg/float64_pair_multi_array.hpp"
#include "task_msg/msg/float64_multi_array.hpp"
#include "task_msg/srv/point2d_move.hpp"
#include "task_msg/srv/nav_find.hpp"
#include "task_msg/srv/path_plan.hpp"

namespace globalPlanningCpp{
    const auto toml_file = toml::parse(ROOT "src/field.toml");
    class GlobalPlanner:public rclcpp::Node{
        public:
        int node_tot;
        std::vector<Eigen::Vector2d> point;
        std::vector<std::vector<int>>  edge;
        std::vector<std::vector<int>>   ver;
        std::priority_queue<std::pair<double,int>> q;
        std::vector<double> d;
        std::vector<bool> v;

        // rclcpp::Client<task_msg::srv::global_able>::SharedPtr client_segment_able;
        rclcpp::Client<task_msg::srv::NavFind>::SharedPtr client_nav_find;
        rclcpp::Client<task_msg::srv::PathPlan>::SharedPtr client_nav_local;
        rclcpp::Service<task_msg::srv::PathPlan>::SharedPtr server_nav_global;
        // rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_pos;
        // rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_global_endpoint2d;
        // rclcpp::Publisher<task_msg::msg::Float64MultiArray>::SharedPtr publisher_local_endpoint2d;
        // rclcpp::TimerBase::SharedPtr timer_;
        
        std::shared_ptr<rclcpp::Node> node_navfind;

        std::shared_ptr<rclcpp::Node> node_global_server;

        Eigen::Vector2d global_end_point,my_position;
        int global_end_point_flag,my_pos_flag;

        void meter_position_callback(task_msg::msg::Float64MultiArray msg){my_position = Eigen::Map<Eigen::Vector2d>(msg.data.data());my_pos_flag = 1;}

        explicit GlobalPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("global_planner", options){
            RCLCPP_INFO(this->get_logger(),"server starting...");
            using namespace std::placeholders;
            node_tot = toml::find<int>(toml_file,"node_tot_num");
            edge = toml::find<std::vector<std::vector<int>>>(toml_file,"edge_able");
            RCLCPP_INFO(this->get_logger(),"server starting...1");
            ver.resize(node_tot);
            for(int i=0;i<node_tot;++i)
                for(int j=0;j<node_tot;++j)
                if(edge[i][j])
                ver[i].push_back(j);
            RCLCPP_INFO(this->get_logger(),"server starting...2");

            auto point_mp = toml::find<std::vector<std::pair<double,double>>>(toml_file,"point_position");
            RCLCPP_INFO(this->get_logger(),"server starting...3");
            point.resize(node_tot);v.resize(node_tot);d.resize(node_tot);
            for(int i=0;i<node_tot;++i)
            point[i] = Eigen::Vector2d(point_mp[i].first,point_mp[i].second),d[i] = -1;
            RCLCPP_INFO(this->get_logger(),"server starting...4");
            //client_segment_able = node->create_client<project_interfaces::srv::global_able>("query_for_segment_able");
            // subscriber_pos = this->create_subscription<task_msg::msg::Float64MultiArray>("meter_position",10,std::bind(&GlobalPlanner::global_meter_position_callback, this, _1));
            // subscriber_global_endpoint2d =this->create_subscription<task_msg::msg::Float64MultiArray>("end_point_global_planner",10,std::bind(&GlobalPlanner::end_point_global_planner_call_back,this,_1));
            node_navfind = rclcpp::Node::make_shared("nav_find_client");
            node_global_server = rclcpp::Node::make_shared("nav_global_server");
            client_nav_find = node_navfind->create_client<task_msg::srv::NavFind>("navgation_pos_find");
            client_nav_local = node_navfind->create_client<task_msg::srv::PathPlan>("local_navigation");
            server_nav_global = node_global_server->create_service<task_msg::srv::PathPlan>("global_navigation",std::bind(&GlobalPlanner::global_direction_service,this,_1,_2));
            // publisher_local_endpoint2d = this->create_publisher<task_msg::msg::Float64MultiArray>("end_point_local_planner",10);
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&GlobalPlanner::global_direction, this));
            global_end_point_flag = 0;
            RCLCPP_INFO(this->get_logger(),"server initialization finished");
            std::thread{std::bind(&GlobalPlanner::nav_global_server_start,this)}.detach();
        }
        void nav_global_server_start(){RCLCPP_INFO(this->get_logger(),"?????");rclcpp::spin(node_global_server);rclcpp::shutdown();}
        void end_point_global_planner_call_back(task_msg::msg::Float64MultiArray msg){
            // RCLCPP_INFO(this->get_logger(),"server enter target callback..");
            global_end_point = Eigen::Map<Eigen::Vector2d>(msg.data.data());
            ++global_end_point_flag;
            // RCLCPP_INFO(this->get_logger(),"server received target");
            return;
        }
        void global_meter_position_callback(task_msg::msg::Float64MultiArray msg){
            // RCLCPP_INFO(this->get_logger(),"server enter position callback..");
            my_position = Eigen::Map<Eigen::Vector2d>(msg.data.data());
            my_pos_flag = 1;
            // RCLCPP_INFO(this->get_logger(),"server turn on the meter callback service!!!!.position:[%lf,%lf]",my_position[0],my_position[1]);
            return;
        }
        std::vector<int> nav_pos_find(Eigen::Vector2d pos,bool flag = 0){
            auto request = std::make_shared<task_msg::srv::NavFind::Request>();
            task_msg::msg::Float64PairMultiArray msg_pair;
            task_msg::msg::Float64Pair one_msg;
            msg_pair.data.resize(node_tot);
            for(int i=0;i<node_tot;++i){
                msg_pair.data[i].x = point[i][0];
                msg_pair.data[i].y = point[i][1];
            }
            if(flag)
            one_msg.x = global_end_point[0],
            one_msg.y = global_end_point[1],
            msg_pair.data.push_back(one_msg);
            std::vector<double> target_point_array;
            target_point_array.push_back(pos[0]);
            target_point_array.push_back(pos[1]);
            request->pos_post = msg_pair;
            request->pos.data = target_point_array;
            while (!client_nav_find->wait_for_service(std::chrono::milliseconds(1000)) && rclcpp::ok());
            auto result = client_nav_find->async_send_request(request);
            rclcpp::spin_until_future_complete(node_navfind,result);
            std::vector<int> ret = result.get()->id;
            return ret;
        }
        Eigen::Vector2d local_direction_get(Eigen::Vector2d old_velo,Eigen::Vector2d local_end_point){
            auto request = std::make_shared<task_msg::srv::PathPlan::Request>();
            request->old_velo.data = std::vector<double>{old_velo[0],old_velo[1]};
            request->end_point.data = std::vector<double>{local_end_point[0],local_end_point[1]};
            request->start_point.data = std::vector<double>{my_position[0],my_position[1]};
            while (!client_nav_local->wait_for_service(std::chrono::milliseconds(1000)) && rclcpp::ok());
            auto result = client_nav_local->async_send_request(request);
            rclcpp::spin_until_future_complete(node_navfind,result);
            std::vector<double> ret = result.get()->velo.data;
            return Eigen::Map<Eigen::Vector2d>(ret.data());
        }
        bool globalPlanning_reload(Eigen::Vector2d end_point){
            // RCLCPP_INFO(this->get_logger(),"reloading...");
            auto able_id = nav_pos_find(end_point);
            if(able_id.empty()) return 0;
            int end_id=able_id[0],end_dis,calc_dis;
            end_dis = (point[able_id[0]]-end_point).norm();
            for(int i=0;i<node_tot;++i) v[i] = 0,d[i]=-1;
            for(int i=1,lim = able_id.size();i<lim;++i){
                calc_dis = (point[able_id[i]]-end_point).norm();
                if(calc_dis < end_dis)
                end_id = able_id[i],
                end_dis = calc_dis;
            }

            d[end_id] = end_dis;
            
            while(!q.empty() && rclcpp::ok()) q.pop();

            q.push(std::make_pair(0,end_id));
            
            while(!q.empty() && rclcpp::ok()){
                auto x = q.top();
                q.pop();
                if(v[x.second]) continue;
                v[x.second]=1;
                // RCLCPP_INFO(this->get_logger(),"%d",x.second);
                for(auto y:ver[x.second]){
                    double dis_xy = d[x.second] + (point[y]-point[x.second]).norm();
                    if(d[y] < 0 || dis_xy < d[y]){
                        d[y] = dis_xy;
                        q.push(std::make_pair(-d[y],y));
                    }
                }
            }
            // RCLCPP_INFO(this->get_logger(),"reloaded.");
            return 1;
        }

        int global_direction_once(Eigen::Vector2d start_point){
            auto able_id = nav_pos_find(start_point,1);
            if(able_id.empty())
            return -1;
            if(able_id.back() == node_tot)
            return -1;
            //能看见就往回走
            int start_id = -1;
            double start_dis = 1e9;
            // RCLCPP_INFO(this->get_logger(),"direction once..");
            // auto request = std::make_shared<project_interfaces::srv::global_able::Request>();
            for(int j=0,lim = able_id.size();j<lim;++j){
                int i = able_id[j];
                if(start_dis > d[i] && (point[i]-start_point).norm() > 0.2){
                    // RCLCPP_INFO(this->get_logger(),"%d %lf",i,d[i]);
                    start_dis = d[i],start_id = i;
                }
            }
            // RCLCPP_INFO(this->get_logger(),"direction once..finish");
            return start_id;
        }

        Eigen::Vector2d global_direction(Eigen::Vector2d old_velo){
            if(!globalPlanning_reload(global_end_point))
            return local_direction_get(old_velo,global_end_point);

            int target_id = -1;
            target_id = global_direction_once(my_position);
            // RCLCPP_INFO(this->get_logger(),"target_id:%d",target_id);
            std::vector<double> local_end_point;local_end_point.resize(2);
            if(~target_id)
                local_end_point[0] = point[target_id][0],
                local_end_point[1] = point[target_id][1];
            else
                local_end_point[0] = global_end_point[0],
                local_end_point[1] = global_end_point[1];

            // RCLCPP_INFO(this->get_logger(),"local_target_point:[%lf,%lf]",local_end_point[0],local_end_point[1]);

            Eigen::Vector2d velo = local_direction_get(old_velo,Eigen::Map<Eigen::Vector2d>(local_end_point.data()));
            return velo;
        }

        void global_direction_service(const std::shared_ptr<task_msg::srv::PathPlan::Request> request,std::shared_ptr<task_msg::srv::PathPlan::Response> response){
            // RCLCPP_INFO(this->get_logger(),"??????");
            my_position = Eigen::Map<Eigen::Vector2d>(request->start_point.data.data());
            global_end_point = Eigen::Map<Eigen::Vector2d>(request->end_point.data.data());
            my_pos_flag = global_end_point_flag = 1;
            Eigen::Vector2d velo = global_direction(Eigen::Map<Eigen::Vector2d>(request->old_velo.data.data()));
            response->velo.data = std::vector<double>{velo[0],velo[1]};
            return;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(globalPlanningCpp::GlobalPlanner)