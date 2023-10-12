#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <toml.hpp>
#include <utility>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include "task_msg/msg/float64_tri_multi_array.hpp"
#include "task_msg/msg/float64_pair_multi_array.hpp"
#include "task_msg/msg/float64_multi_array.hpp"
#include "task_msg/srv/point2d_move.hpp"
#include "task_msg/srv/nav_find.hpp"
#include "task_msg/srv/path_plan.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//仿真包括几个部分：
//模拟球体与模拟长方体
//其中模拟球体用来模拟车辆，子弹与地脉镇石
//模拟长方体用来模拟墙体

//这些类相互之间有派生关系

//节点利用回调函数，每个周期内做一次子弹的模拟运动与结算。


//墙壁，自机，子弹，镇石分别对应的为Wall，infantry，little_bullet，infantry

namespace platform{
    int object_tot = 0;
    const auto toml_file = toml::parse(ROOT "src/config.toml");
    class object_group:public rclcpp::Node{
        public:

        //基类

        class object{
            public:
            int id;
            Eigen::Vector3d position;
            //this can be corrected by adding ros::Time later.
            double weight;
            object(Eigen::Vector3d position_,double weight_ = 0):position(position_),weight(weight_){
                id = ++object_tot;
            }
        };

        //大部分三维场景最后都改为了二维

        //长方体基类

        class Cuboid:public object{
            public:
            Eigen::Vector3d norm;
            double height;
            double width;
            double length;
            std::vector<Eigen::Vector3d> angle_dot;

            void refresh_angle_dot();//获取角点，四个角点存在方位，没有使用opencv的rotate rectangle

            Cuboid(Eigen::Vector3d position_,Eigen::Vector3d norm_ = Eigen::Vector3d(0.,0.,0.),double height_ = 0,double width_ = 0,double length_ = 0):
            object(position_,-1),norm(norm_),height(height_),width(width_),length(length_){
                refresh_angle_dot();
            }
            
            template<class T>
            bool judge_inside(T target);
        };

        //球体基类
    
        class Sphere:public object{
            public:
            double radius;
            double ang_velo;
            Eigen::Vector3d velo;
            Eigen::Vector3d velo_;
            bool updater;
            int type;
            Sphere(Eigen::Vector3d position_,double weight_ = 0,double radius_ = 0,Eigen::Vector3d velo_init = Eigen::Vector3d(0.,0.,0.)):object(position_,weight_),radius(radius_){
                velo = velo_init;
                velo_ = velo_init;
                ang_velo = 0;
                updater = 0;
            }
            void cope_collision(Sphere target);//碰撞检测，将碰撞切面法向量方向的速度置零，也即认为是完全非弹性碰撞
            void cope_collision(Cuboid target,int flag);//下面几个区别不大，与长方体的碰撞带flag是为了检测是哪一面
            bool judge_collision(Sphere target);
            bool judge_collision(Cuboid target,bool cope_flag);
            bool judge_collision_with_grid(bool cope_flag);
            void interval_move(double dT);//执行一次移动
            void update();//如果一次碰到了两个物体，速度就会为0
        };

        //场地
        
        class Wall:public Cuboid{
            public:
            double A1,B1,C1,A2,B2,C2;
            double margin1,margin2,calc1,calc2;
            //以上两行系数都是局部规划的避障函数。
            Wall():Cuboid(Eigen::Vector3d(0.,0.,0.),Eigen::Vector3d(0.,0.,0.),0.,0.,0.),margin1(0.),margin2(0.){;}
            Wall(Eigen::Vector3d position_,Eigen::Vector3d norm_,double height_ = 0,double width_ = 0,double length_ = 0,double margin1_ = 0,double margin2_ = 0):
            Cuboid(position_,norm_,height_,width_,length_),margin1(margin1_),margin2(margin2_){}
            Eigen::Vector2d func(Eigen::Vector2d vec);//局部规划避障函数，弃用
            Eigen::Vector3d push(Eigen::Vector3d ori);//局部规划避障推点函数，弃用
        };

        class any_bullet:public Sphere{
            int health_point_damage;
            public:
            int get_health_point_damage(){return health_point_damage;}
            any_bullet(Eigen::Vector3d position_,double weight_ = 0,double radius_ = 0,int health_point_damage_ = 0):Sphere(position_,weight_,radius_),health_point_damage(health_point_damage_){;}
        };

        //场地中车的实体

        class any_car:public Sphere{
            public:
            int health_point;
            Eigen::Vector2d target;
            //
            double C,B,A;
            double margin1,margin2;
            //系数
            //当时的想法是：将距离margin2内的局部规划目标点全部映射到margin1与margin2内，模拟ros_teb但只模拟一个点来达到效果
            //后来发现效果不好，留给你们优化了
            int health_point_limit,target_id;
            public:
            any_car(Eigen::Vector3d position_,double radius_ = 0,double margin1_ = 0,double margin2_ = 0,double weight_ = 0,int health_point_limit_ = 0):Sphere(position_,weight_,radius_),margin1(margin1_),margin2(margin2_),health_point_limit(health_point_limit_){
                double calc1,calc2;
                calc1 = radius + margin1;
                calc2 = calc1+margin2;
                A = calc1/calc2/calc2;
                B = 1-2*calc1/calc2;
                C = calc1;
                target = Eigen::Vector2d(-1,-1);
                health_point = health_point_limit;
            }
            double func(double X){if(X>=radius+margin1+margin2) return radius+margin1+margin2;return A*X*X + B*X + C;}
            Eigen::Vector3d push(Eigen::Vector3d ori);
            //用来推点的函数

            void set_health_point(int health_point_){
                health_point = std::min(health_point_limit,health_point_);
            }
            void set_health_point_limit(int health_point_limit_){
                health_point_limit = health_point_limit_;
                set_health_point(health_point);
                return;
            }
        };

        //场地弹丸实体

        class little_bullet:public any_bullet{
            public:
            little_bullet(Eigen::Vector3d position_):any_bullet(position_,0.05,bullet_radius,10){}
        };

        //场地步兵实体

        class infantry:public any_car{
            public:
            infantry(Eigen::Vector3d position_):
            any_car(position_,toml::find<double>(toml_file,"car_radius"),
                    toml::find<double>(toml_file,"car_margin1"),
                    toml::find<double>(toml_file,"car_margin2"),10000,
                    toml::find<int>(toml_file,"infantry_hp")){type=0;shooter_able=0;shooting_angle=0;}
            infantry():
            any_car(Eigen::Vector3d(0.,0.,0.),toml::find<double>(toml_file,"car_radius"),
                    toml::find<double>(toml_file,"car_margin1"),
                    toml::find<double>(toml_file,"car_margin2"),10000,
                    toml::find<int>(toml_file,"infantry_hp")){type=0;shooter_able=0;shooting_angle=0;}
            
            int shooter_able;
            double shooting_angle;
            void cause_damage(little_bullet bullet,int id = 0){//造成伤害
                health_point -= bullet.get_health_point_damage();
                health_point = std::max(health_point,0);
                if(id>=0){
                    target_id = id;
                }
                return;
            }
            void add_health_point(int health_point_){//增加血量
                health_point = std::min(health_point_limit,health_point_ + health_point);
            }
        };


        //场地变量
        //player_infantry为玩家操控角色
        //其余vector存储场地对象
        //我真应该用C#

        std::vector<little_bullet>  little_bullet_queue;
        std::vector<infantry>       infantry_queue;
        std::vector<Wall>           Wall_queue;
        std::vector<infantry>       Stone_queue;
        std::vector<infantry>       enemy_vis_queue;
        infantry                    player_infantry;
        inline static std::vector<Wall>           Wall_grid[50][50];
        inline static double bullet_radius;
        double map_height_platform;
        double map_width_platform;
        double enemy_speed_limit;
        double my_speed_limit;
        
        
        //与整个框架相关，级别为场地的对象操作
        
        inline static const int TS_rate = toml::find<double>(toml_file,"TS_rate");
        inline static const int recheck_ti = 5;
        void recheck();
        void start_init();
        bool put_soldier(Eigen::Vector3d position);
        infantry get_soldier();
        void refresh();
        Eigen::Vector3d push_point(Eigen::Vector3d ori);
        void server_pusher(const std::shared_ptr<task_msg::srv::Point2dMove::Request> request,std::shared_ptr<task_msg::srv::Point2dMove::Response>      response);
        void timer_callback();
        void drawing_field();
        void create_shooting(infantry element,double angle);
        bool segment_cross(std::pair<Eigen::Vector2d,Eigen::Vector2d> segment1,
                            std::pair<Eigen::Vector2d,Eigen::Vector2d> segment2);
        bool check_if_able(Eigen::Vector2d start_point,Eigen::Vector2d end_point);

        //与墙是否相交

        //通信部分变量

        rclcpp::Service<task_msg::srv::Point2dMove>::SharedPtr service_point2dmove;
        rclcpp::Service<task_msg::srv::NavFind>::SharedPtr service_navfind;
        rclcpp::Client<task_msg::srv::PathPlan>::SharedPtr client_nav_global;
        rclcpp::Publisher<task_msg::msg::Float64MultiArray>::SharedPtr publisher_velo;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_ang_velo;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_self_blood;
        rclcpp::Publisher<task_msg::msg::Float64MultiArray>::SharedPtr publisher_pos;
        rclcpp::Publisher<task_msg::msg::Float64TriMultiArray>::SharedPtr publisher_enemy_info;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_local_velo;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_local_ang_velo;
        rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_global_plan_pos;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_shooting_angle;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_shooter_able;
        rclcpp::TimerBase::SharedPtr timer_,timer2_,timer3_,timer4_;
        std::shared_ptr<rclcpp::Node> node_movepoint;
        std::shared_ptr<rclcpp::Node> node_nav_global;

        //用以通信的函数与回调操作

        void global_plan_pos_receive_callback(task_msg::msg::Float64MultiArray msg){player_infantry.target = Eigen::Map<Eigen::Vector2d>(msg.data.data());}
        void local_velocity_receive_callback(const std_msgs::msg::Float64 &msg){player_infantry.velo[1] = msg.data;}
        void local_angular_velocity_receive_callback(const std_msgs::msg::Float64 &msg){player_infantry.ang_velo = msg.data;}
        void decision_shooter_able_receive_callback(const std_msgs::msg::Int64 &msg){player_infantry.shooter_able = msg.data;}
        void decision_shooting_angle_receive_callback(const std_msgs::msg::Float64 &msg){player_infantry.shooting_angle = msg.data;}
        void pointmove_server_start(){RCLCPP_INFO(this->get_logger(),"test me...");rclcpp::spin(node_movepoint);rclcpp::shutdown();}//局部规划避障服务，现已不用，可自行修改srv与这个服务
        void server_locator(const std::shared_ptr<task_msg::srv::NavFind::Request> request,std::shared_ptr<task_msg::srv::NavFind::Response>      response);//检测pos_post中哪些点在pos的视野范围内，可以直接到达而不用穿墙
        void publish_detector();//检测
        Eigen::Vector2d get_global_navigation_velo(Eigen::Vector2d old_velo,Eigen::Vector2d start_point,Eigen::Vector2d end_point);
        
        //目前的导航如下：
        //场地发起导航请求到全局，全局定好局部终点后发到局部，局部返回速度信息，全局再将速度信息返回场地

        //决策如下：
        //你发出的决策导航信息会被player_infantry的target的接受，作为此后的目标点
        //你发出的发弹信息会进入player_infantry中执行发弹指令。

        //实例化节点
        explicit object_group(const rclcpp::NodeOptions & options)
            : Node("platform", options){
            RCLCPP_INFO(this->get_logger(),"platform starting...");
            using namespace std::placeholders;
            map_height_platform = toml::find<double>(toml_file,"map_height_platform");
            map_width_platform = toml::find<double>(toml_file,"map_width_platform");
            enemy_speed_limit = toml::find<double>(toml_file,"VERTICAL_ENEMY");
            my_speed_limit = toml::find<double>(toml_file,"VERTICAL_LIMIT");
            bullet_radius = toml::find<double>(toml_file,"BULLET_RADIUS");
            start_init();
            node_movepoint = rclcpp::Node::make_shared("point_mover_server");
            node_nav_global = rclcpp::Node::make_shared("navigation_global_client");
            player_infantry = get_soldier();
            player_infantry.type = 1;
            service_point2dmove = node_movepoint->create_service<task_msg::srv::Point2dMove>("point_mover",std::bind(&object_group::server_pusher,this,_1,_2));
            service_navfind = node_movepoint->create_service<task_msg::srv::NavFind>("navgation_pos_find",std::bind(&object_group::server_locator,this,_1,_2));
            client_nav_global = node_nav_global->create_client<task_msg::srv::PathPlan>("global_navigation");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&object_group::timer_callback, this));
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&object_group::drawing_field, this));
            timer3_ = this->create_wall_timer(std::chrono::milliseconds(recheck_ti),std::bind(&object_group::recheck,this));
            timer4_ = this->create_wall_timer(std::chrono::milliseconds(1000*toml::find<int>(toml_file,"ENEMY_SPAWN_INTERVAL")),std::bind(&object_group::refresh,this));
            publisher_velo = this->create_publisher<task_msg::msg::Float64MultiArray>("meter_velocity", 10);
            publisher_ang_velo = this->create_publisher<std_msgs::msg::Float64>("meter_angular_velocity", 10);
            publisher_pos = this->create_publisher<task_msg::msg::Float64MultiArray>("meter_position", 10);
            publisher_enemy_info = this->create_publisher<task_msg::msg::Float64TriMultiArray>("enemy_detect_pos",10);
            publisher_self_blood = this->create_publisher<std_msgs::msg::Float64>("self_health_point",10);
            subscriber_global_plan_pos = this->create_subscription<task_msg::msg::Float64MultiArray>("end_point_global_planner",10,std::bind(&object_group::global_plan_pos_receive_callback,this,_1));
            subscriber_local_velo = this->create_subscription<std_msgs::msg::Float64>("local_velocity",10,std::bind(&object_group::local_velocity_receive_callback,this,_1));
            subscriber_local_ang_velo = this->create_subscription<std_msgs::msg::Float64>("local_angular_velocity",10,std::bind(&object_group::local_angular_velocity_receive_callback,this,_1));
            subscriber_shooting_angle = this->create_subscription<std_msgs::msg::Float64>("decision_shooting_angle",10,std::bind(&object_group::decision_shooting_angle_receive_callback,this,_1));
            subscriber_shooter_able = this->create_subscription<std_msgs::msg::Int64>("decision_shooting_able",10,std::bind(&object_group::decision_shooter_able_receive_callback,this,_1));
            std::thread{std::bind(&object_group::pointmove_server_start, this)}.detach();
        }
    };
}