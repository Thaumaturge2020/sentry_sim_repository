#include "platform/object.hpp"

//cpp中的函数都是hpp中的实例化，请不要看这里来理解架构，看hpp

namespace platform{

    void object_group::Cuboid::refresh_angle_dot(){
        angle_dot.clear();
        Eigen::Vector3d vec = norm;
        // RCLCPP_INFO(rclcpp::get_logger("refresh_angle_dot"),"norm:[%lf,%lf]",vec[0],vec[1]);
        std::swap(vec[0],vec[1]);
        vec[1] = -vec[1];
        Eigen::Vector3d mid_dot1 = position + vec * length / 2;
        Eigen::Vector3d mid_dot2 = position - vec * length / 2;
        angle_dot.push_back(mid_dot1 + norm*width/2);
        angle_dot.push_back(mid_dot1 - norm*width/2);
        angle_dot.push_back(mid_dot2 - norm*width/2);
        angle_dot.push_back(mid_dot2 + norm*width/2);
        // RCLCPP_INFO(rclcpp::get_logger("refresh_angle_dot"),"mid_dot1:[%lf,%lf]",mid_dot1[0],mid_dot1[1]);
        // RCLCPP_INFO(rclcpp::get_logger("refresh_angle_dot"),"mid_dot2:[%lf,%lf]",mid_dot2[0],mid_dot2[1]);
        return;
    }

    Eigen::Vector3d object_group::Wall::push(Eigen::Vector3d ori){//一次局部路径规划避障点“推移”，将目标点在墙的两个方向向量上进行分解，然后推到厚度为margin1+margin2的边缘处
        Eigen::Vector2d tar2d;
        Eigen::Vector3d tar,vec;
        RCLCPP_INFO(rclcpp::get_logger("wall_pusher_before_before"),"%lf %lf",ori[0],ori[1]);
        tar = ori-position;
        RCLCPP_INFO(rclcpp::get_logger("wall_pusher_before"),"%lf %lf",tar[0],tar[1]);
        tar2d[1] = tar.dot(norm)/norm.norm();
        vec = norm;std::swap(vec[0],vec[1]);vec[0] = -vec[0];
        tar2d[0] = tar.dot(vec)/vec.norm();
        
        RCLCPP_INFO(rclcpp::get_logger("wall_pusher_before_after"),"%lf %lf %lf %lf %lf %lf",tar2d[0],tar2d[1],vec[0],vec[1],vec[2],tar.dot(vec));

        bool flagX = (tar2d[0]<0),flagY = (tar2d[1]<0);
        tar2d[0] = (flagX?-tar2d[0]:tar2d[0]);
        tar2d[1] = (flagY?-tar2d[1]:tar2d[1]);
        
        tar2d = func(tar2d);
        tar2d[0] = (flagX?-tar2d[0]:tar2d[0]);
        tar2d[1] = (flagY?-tar2d[1]:tar2d[1]);

        RCLCPP_INFO(rclcpp::get_logger("wall_pusher_after_before"),"%lf %lf",tar2d[0],tar2d[1]);
        
        tar = tar2d[1] * norm / norm.norm() + tar2d[0] * vec / vec.norm() + position;
        RCLCPP_INFO(rclcpp::get_logger("wall_pusher_after"),"%lf %lf",tar[0],tar[1]);
        return tar;
    }

    Eigen::Vector3d object_group::any_car::push(Eigen::Vector3d ori){//一次局部路径规划避障点“推移”，因为无需像长方体那样考虑两方面所
        double len;
        Eigen::Vector3d tar,vec;
        tar = ori-position;
        len = tar.norm();
        len = func(len);
        tar = len*tar/tar.norm() + position;
        return tar;
    }

    Eigen::Vector3d object_group::push_point(Eigen::Vector3d ori){//局部路径规划避障点全推移，但这是错的
        double x_now = ori[0],y_now = ori[1];
        int X_now = floor(x_now),Y_now = floor(y_now);
        for(auto i:Wall_grid[X_now][Y_now]){
            ori = i.push(ori);
        }
        for(auto i:infantry_queue){
            ori = i.push(ori);
        }
        return ori;
    }

    void object_group::server_pusher(const std::shared_ptr<task_msg::srv::Point2dMove::Request> request,std::shared_ptr<task_msg::srv::Point2dMove::Response>      response)//局部规划避障点推移的服务端
    {
        auto ori_ei = Eigen::Vector3d(request->ori.data[0],request->ori.data[1],0.);
        RCLCPP_INFO(rclcpp::get_logger("server_pusher"), "receiving request: [(%lf,%lf)]", (double)ori_ei[0],(double)ori_ei[1]);
        auto tar_ei = push_point(ori_ei);
        std::vector<double> tar;
        tar.resize(2);tar[0] = tar_ei[0],tar[1] = tar_ei[1];
        response->tar.data = tar;
        RCLCPP_INFO(rclcpp::get_logger("server_pusher"), "sending back response: [(%lf,%lf)]", (double)tar[0],(double)tar[1]);
        return;
    }

    template<class T>
    bool object_group::Cuboid::judge_inside(T target){//判断一个长方体目标是否与另一个长方体有交
        auto    dot_array_1 = angle_dot,
                dot_array_2 = target.angle_dot;

        dot_array_1.push_back(dot_array_1[0]);
        dot_array_2.push_back(dot_array_2[0]);

        double area1 = 0;
        double area2 = 0;

        Eigen::Vector3d ori,tar1,tar2;

        ori = dot_array_1[0];
        for(int i=0,lim1 = dot_array_1.size();i<lim1-1;++i){
            tar1 = dot_array_1[i  ] - ori;
            tar2 = dot_array_1[i+1] - ori;
            area1 += tar1[0]*tar2[1] - tar2[0]*tar1[1];
        }
        area1 = fabs(area1);

        ori = dot_array_2[0];
        for(int i=0,lim2 = dot_array_2.size();i<lim2-1;++i){
            tar1 = dot_array_2[i  ] - ori;
            tar2 = dot_array_2[i+1] - ori;
            area2 += tar1[0]*tar2[1] - tar2[0]*tar1[1];
        }
        area2 = fabs(area2);
        
        for(int i=0,lim1 = dot_array_1.size();i<lim1-1;++i){
            double area = 0;
            ori = dot_array_1[i];
            for(int j=0,lim2 = dot_array_2.size();j<lim2-1;++j){
                tar1 = dot_array_2[j  ] - ori;
                tar2 = dot_array_2[j+1] - ori;
                area += fabs(tar1[0]*tar2[1] - tar2[0]*tar1[1]);
            }
            area = fabs(area);
            if(fabs(area - area2)/area2 < 1e-3)
            return true;
        }

        for(int i=0,lim1 = dot_array_2.size();i<lim1-1;++i){
            double area = 0;
            ori = dot_array_2[i];
            for(int j=0,lim2 = dot_array_1.size();j<lim2-1;++j){
                tar1 = dot_array_1[j  ] - ori;
                tar2 = dot_array_1[j+1] - ori;
                area += fabs(tar1[0]*tar2[1] - tar2[0]*tar1[1]);
            }
            area = fabs(area);
            if(fabs(area - area1)/area1 < 1e-3)
            return true;
        }
        return false;
    }

    void object_group::Sphere::cope_collision(object_group::Sphere target){//解决球体碰撞
        Eigen::Vector3d dis_shift = position - target.position;
        Eigen::Vector3d self_velo = Eigen::Vector3d(cos(velo[0]) * velo[1],sin(velo[0]) * velo[1],0);
        double vertical_shift = dis_shift.norm();
        vertical_shift = std::max(vertical_shift,radius + target.radius);
        // Eigen::Vector3d target_velo = Eigen::Vector3d(cos(target.velo[0]) * target.velo[1],sin(target.velo[0]) * target.velo[1],0);
        double vertical_self_velo = self_velo.dot(dis_shift) / dis_shift.norm();
        if(vertical_self_velo > 0) vertical_self_velo = 0;
        // double vertical_target_velo = target_velo.dot(dis_shift) / dis_shift.norm();
        // double my_velo = vertical_self_velo * (weight - target.weight)/(weight + target.weight) + vertical_target_velo * (2*target.weight)/(weight+target.weight);
        Eigen::Vector3d velo_now = self_velo + vertical_self_velo * dis_shift / dis_shift.norm();
        
        position = target.position + vertical_shift * dis_shift/dis_shift.norm();

        if(velo_now.norm() > 1e-12 && updater == 0)
        velo = Eigen::Vector3d(atan2(velo_now[1],velo_now[0]),velo_now.norm(),0),updater = 1;
        else
        velo[1] = 0;
        return;
    }
    void object_group::Sphere::cope_collision(object_group::Cuboid target,int flag){//解决球体与长方体的碰撞
        Eigen::Vector3d vec = target.norm;
        Eigen::Vector3d self_velo = Eigen::Vector3d(cos(velo[0]) * velo[1],sin(velo[0]) * velo[1],0);
        Eigen::Vector3d self_pos = position - target.position;

        int FLAG = flag;

        while(flag)
        std::swap(vec[0],vec[1]),vec[0] = -vec[0],--flag;
        double vertical_self_velo = self_velo.dot(vec)/vec.norm();
        double vertical_pos = self_pos.dot(vec)/vec.norm();
        Eigen::Vector3d ret_pos = self_pos - vertical_pos*vec/vec.norm();

        // RCLCPP_INFO(rclcpp::get_logger("cope_collision"),"info before %lf %lf %lf %lf %lf",vertical_pos,target.length/2,target.width/2,vec[0],vec[1]);

        if(FLAG&1)  vertical_pos = std::max(vertical_pos,target.length/2 + radius);
        else        vertical_pos = std::max(vertical_pos,target.width/2 + radius);

        // RCLCPP_INFO(rclcpp::get_logger("cope_collision"),"info after %lf %lf %lf %lf %lf",vertical_pos,target.length/2,target.width/2,vec[0],vec[1]);

        if(vertical_self_velo > 0) vertical_self_velo = 0;
        
        Eigen::Vector3d velo_now = self_velo + vertical_self_velo*vec/vec.norm();
        position = vertical_pos*vec/vec.norm() + ret_pos + target.position;
    
        if(velo_now.norm() > 1e-12)
        velo = Eigen::Vector3d(atan2(velo_now[1],velo_now[0]),velo_now.norm(),0),updater = 1;
        else
        velo[1] = 0;
        return;
    }

    bool object_group::Sphere::judge_collision(object_group::Sphere target){
        double dist_limit = radius + target.radius;
        Eigen::Vector3d dis_shift = target.position - position;
        
        if(dis_shift.norm() <= dist_limit){
            cope_collision(target);
            return true;
        }
        return false;
    }

    bool object_group::Sphere::judge_collision(object_group::Cuboid target,bool cope_flag = 1){
        // bool flag = 0;
        double dist_limit = radius;

        Eigen::Vector3d vec = target.norm;
        std::swap(vec[0],vec[1]);
        vec[0] = -vec[0];
        
        Eigen::Vector3d my_dot1 = target.position - vec * target.length / 2;
        Eigen::Vector3d my_dot2 = target.position + vec * target.length / 2;
        std::vector<Eigen::Vector3d> dot;
        dot.push_back(my_dot1 + target.norm * target.width/2);
        dot.push_back(my_dot2 + target.norm * target.width/2);
        dot.push_back(my_dot2 - target.norm * target.width/2);
        dot.push_back(my_dot1 - target.norm * target.width/2);
        dot.push_back(my_dot1 + target.norm * target.width/2);

        double dis1,dis = 1e9;
        //RCLCPP_INFO(rclcpp::get_logger("prepare_to_collision"),"%lf %lf",target.position[0],target.position[1]);
        int count = 0;
        for(int i=0;i<4;++i){
            Eigen::Vector3d dot1 = (dot[i]-position);
            Eigen::Vector3d dot2 = (dot[i+1]-position);
            Eigen::Vector3d dot3 = (i&1?vec:target.norm);
            dis = dot1.norm();
            // RCLCPP_INFO(rclcpp::get_logger("judge_collision_once"),"%d %lf %lf",i,dis,dist_limit);
            if((dot3[0]*dot1[1]-dot3[1]*dot1[0])*(dot3[0]*dot2[1]-dot3[1]*dot2[0])<=0){
                dis1 = ((dot1+dot2)/2).dot(dot3) / dot3.norm();
                dis1 = fabs(dis1);
                ++count;
                // RCLCPP_INFO(rclcpp::get_logger("judge_collision_once"),"%d %lf %lf",i,dis1,dist_limit);
                if(dis1 < dis) dis = dis1;
            }
            if(dis <= dist_limit){
                if(cope_flag) cope_collision(target,i);
                return true;
            }
        }
        if(count == 4)
        return true;
        return false;
    }

    bool object_group::Sphere::judge_collision_with_grid(bool cope_flag = 1){
        double  x_min = position[0]-radius,
                x_max = position[0]+radius,
                y_min = position[1]-radius,
                y_max = position[1]+radius;
        
        //RCLCPP_INFO(rclcpp::get_logger("judge_collision_with_grid"),"%lf %lf",position[0],position[1]);
        
        int X_min = floor(x_min),X_max = ceil(x_max)+1,Y_min = floor(y_min),Y_max = ceil(y_max)+1;
        if(X_max == X_min) ++X_max;
        if(Y_max == Y_min) ++Y_max;
        if(X_min > x_min) RCLCPP_ERROR(rclcpp::get_logger("judge_collision_with_grid"),"what the fuck is it X_min!!!!");
        if(X_max < x_max) RCLCPP_ERROR(rclcpp::get_logger("judge_collision_with_grid"),"what the fuck is it X_max!!!!");
        if(Y_min > y_min) RCLCPP_ERROR(rclcpp::get_logger("judge_collision_with_grid"),"what the fuck is it Y_min!!!!");
        if(Y_max < y_max) RCLCPP_ERROR(rclcpp::get_logger("judge_collision_with_grid"),"what the fuck is it Y_max!!!!");

        //RCLCPP_INFO(rclcpp::get_logger("judge_collision_with_grid_border"),"%d %d %d %d",std::max(X_min,0),std::min(X_max,50),std::max(Y_min,0),std::min(Y_max,50));


        for(int i=std::max(X_min,0),lim1 = std::min(X_max,50);i<lim1;++i){
            for(int j=std::max(Y_min,0),lim2 = std::min(Y_max,50);j<lim2;++j)
                for(auto k:Wall_grid[i][j])
                if(judge_collision(k,cope_flag))
                return true;
        }
        return false;
    }

    
    void object_group::Sphere::interval_move(double dT){
        #define pi 3.14159265358979
        //RCLCPP_INFO(rclcpp::get_logger("interval_move"),"this %lf [%lf,%lf] [%lf,%lf]",dT,cos(velo[0])*velo[1],sin(velo[0])*velo[1],velo[0],velo[1]);
        position += Eigen::Vector3d(cos(velo[0])*velo[1],sin(velo[0])*velo[1],0.)*dT;
        if(fabs(ang_velo) < 0.01) return;
        if(ang_velo > 6.28) ang_velo = 6.28;
        if(ang_velo < -6.28) ang_velo = -6.28;

        velo[0] += ang_velo*dT;
        while(velo[0] < -pi)    velo[0] += 2*pi;
        while(velo[0] > pi)     velo[0] -= 2*pi;
        #undef pi
        return;
    }
    
    void object_group::Sphere::update(){
        updater = 0;
        return;
    }

    Eigen::Vector2d object_group::Wall::func(Eigen::Vector2d vec){
        if(vec[0] >= length/2 + margin1 + margin2 || vec[1] >= width/2 + margin1 + margin2) return vec;
        Eigen::Vector2d judge = Eigen::Vector2d(length,width);
        if(judge[0]*vec[1] - judge[1]*vec[0] >= 0){
            calc1 = vec[0]/vec[1]*(width/2 + margin1);
            calc2 = vec[0]/vec[1]*(width/2 + margin1 + margin2);
            A1 = calc1/calc2/calc2;
            B1 = 1-2*calc1/calc2;
            C1 = calc1;
            vec[0] = A1 * vec[0] * vec[0] + B1 * vec[0] + C1;                    
            calc1 = (width/2 + margin1);
            calc2 = (width/2 + margin1 + margin2);
            A1 = calc1/calc2/calc2;
            B1 = 1-2*calc1/calc2;
            C1 = calc1;
            vec[1] = A1 * vec[1] * vec[1] + B1 * vec[1] + C1;
        }
        else{
            calc1 = vec[1]/vec[0]*(length/2 + margin1);
            calc2 = vec[1]/vec[0]*(length/2 + margin1 + margin2);
            A1 = calc1/calc2/calc2;
            B1 = 1-2*calc1/calc2;
            C1 = calc1;
            vec[1] = A1 * vec[1] * vec[1] + B1 * vec[1] + C1;                    
            calc1 = (length/2 + margin1);
            calc2 = (length/2 + margin1 + margin2);
            A1 = calc1/calc2/calc2;
            B1 = 1-2*calc1/calc2;
            C1 = calc1;
            vec[0] = A1 * vec[0] * vec[0] + B1 * vec[0] + C1;
        }
        return vec;
    }

    void object_group::create_shooting(infantry element,double angle){
        little_bullet my_bullet = little_bullet(element.position + Eigen::Vector3d(cos(angle)*(element.radius+0.2),sin(angle)*(element.radius+0.2),0));
        my_bullet.velo[0] = angle;
        my_bullet.velo[1] = 7;
        little_bullet_queue.push_back(my_bullet);
        return;
    }

    void object_group::recheck(){//地图刷新一次
        double dT = recheck_ti/1000.;
        
        static int ENEMY_SHOOT_INTERVAL_NUM = toml::find<int>(toml_file,"ENEMY_SHOOT_INTERVAL_NUM");
        static int SHOOT_INTERVAL_NUM = toml::find<int>(toml_file,"SHOOT_INTERVAL_NUM");
        static int CNT = 0;
        ++CNT;

        bool able_shoot = (CNT%SHOOT_INTERVAL_NUM == 0);
        bool enemy_able_shoot = (CNT%ENEMY_SHOOT_INTERVAL_NUM == 0);

        player_infantry.judge_collision_with_grid();

        for(int j=0,lim2 = infantry_queue.size();j<lim2;++j)
            infantry_queue[j].judge_collision_with_grid();
        
        for(int j=0,lim2 = infantry_queue.size();j<lim2;++j)
            infantry_queue[j].judge_collision_with_grid();

        for(int i=0,lim1 = infantry_queue.size();i<lim1;++i){
            player_infantry.judge_collision(infantry_queue[i]);
            for(int j=0,lim2 = infantry_queue.size();j<lim2;++j)
            if(j!=i)
            infantry_queue[j].judge_collision(infantry_queue[i]);

            for(int j=0,lim2 = Stone_queue.size();j<lim2;++j){
                infantry_queue[i].judge_collision(Stone_queue[j]);
            }
        }

        for(int j=0,lim1 = infantry_queue.size();j<lim1;++j){
            infantry_queue[j].position[0] = std::min(infantry_queue[j].position[0],map_width_platform);
            infantry_queue[j].position[1] = std::min(infantry_queue[j].position[1],map_height_platform);
            infantry_queue[j].position[0] = std::max(infantry_queue[j].position[0],0.);
            infantry_queue[j].position[1] = std::max(infantry_queue[j].position[1],0.);
        }

        player_infantry.position[0] = std::min(player_infantry.position[0],map_width_platform);
        player_infantry.position[1] = std::min(player_infantry.position[1],map_height_platform);
        player_infantry.position[0] = std::max(player_infantry.position[0],0.);
        player_infantry.position[1] = std::max(player_infantry.position[1],0.);

        for(int j=0,lim2 = little_bullet_queue.size();j<lim2;++j)
        if( little_bullet_queue[j].position[0] <= 0 || little_bullet_queue[j].position[1] <= 0 ||
            little_bullet_queue[j].position[0] >= map_width_platform || little_bullet_queue[j].position[1] >= map_height_platform){
                std::swap(little_bullet_queue[j],little_bullet_queue[lim2-1]);
                little_bullet_queue.pop_back();
                --lim2;--j;
            }

        for(int j=0,lim2 = little_bullet_queue.size();j<lim2;++j)
            if(little_bullet_queue[j].judge_collision_with_grid()){
                std::swap(little_bullet_queue[j],little_bullet_queue[lim2-1]);
                little_bullet_queue.pop_back();
                --lim2;--j;
            }

        for(int i=0,lim1 = little_bullet_queue.size();i<lim1;++i){
            for(int j=0,lim2 = infantry_queue.size();j<lim2;++j)
                if(little_bullet_queue[i].judge_collision(infantry_queue[j])){
                    infantry_queue[j].cause_damage(little_bullet_queue[i]);                    
                    std::swap(little_bullet_queue[i],little_bullet_queue[lim1-1]);
                    little_bullet_queue.pop_back();
                    --lim1;--i;
                    break;
                }
            
            if(little_bullet_queue[i].judge_collision(player_infantry)){
                player_infantry.cause_damage(little_bullet_queue[i]);
                std::swap(little_bullet_queue[i],little_bullet_queue[lim1-1]);
                    little_bullet_queue.pop_back();
                    --lim1;--i;
                    continue;
            }
        }
        
        for(int i=0,lim1 = little_bullet_queue.size();i<lim1;++i)
            for(int j=0,lim2 = Stone_queue.size();j<lim2;++j)
                if(little_bullet_queue[i].judge_collision(Stone_queue[j])){
                    Stone_queue[j].cause_damage(little_bullet_queue[i]);
                    std::swap(little_bullet_queue[i],little_bullet_queue[lim1-1]);
                    little_bullet_queue.pop_back();
                    --lim1;--i;
                    break;
                }

        for(int j=0,lim2 = infantry_queue.size();j<lim2;++j){
            if(infantry_queue[j].health_point == 0){
                std::swap(infantry_queue[j],infantry_queue[lim2-1]);
                infantry_queue.pop_back();
                --lim2;--j;
                continue;
            }
            infantry_queue[j].update();
        }

        for(int j=0,lim2 = Stone_queue.size();j<lim2;++j){
            if(Stone_queue[j].health_point == 0){
                std::swap(Stone_queue[j],Stone_queue[lim2-1]);
                Stone_queue.pop_back();
                --lim2;--j;
                continue;
            }
        }

        if(player_infantry.health_point == 0){
            RCLCPP_ERROR(this->get_logger(),"You Died!");
            rclcpp::shutdown();
        }

        if(Stone_queue.empty()){
            RCLCPP_ERROR(this->get_logger(),"You loss!");
            rclcpp::shutdown();
        }

        player_infantry.update();

        
        for(int i=0,lim1 = infantry_queue.size();i<lim1;++i){
            infantry_queue[i].velo[1] = std::min(infantry_queue[i].velo[1],enemy_speed_limit);
            infantry_queue[i].interval_move(dT);
        }

        for(int i=0,lim1 = little_bullet_queue.size();i<lim1;++i){
            little_bullet_queue[i].interval_move(dT);
        }

        player_infantry.interval_move(dT);
        player_infantry.velo[1] = std::min(player_infantry.velo[1],my_speed_limit);

        for(int i=0,lim1 = infantry_queue.size();i<lim1;++i){
            infantry_queue[i].update();
        }

        player_infantry.update();
        if(player_infantry.target[0]>=0 && player_infantry.target[1]>=0){
            Eigen::Vector2d ret = get_global_navigation_velo(
                Eigen::Vector2d(player_infantry.velo[0],player_infantry.velo[1]),
                Eigen::Vector2d(player_infantry.position[0],player_infantry.position[1]),
                player_infantry.target
            );

            player_infantry.velo[1] = ret[1];
            player_infantry.ang_velo = ret[0];
        }

        for(int i=0,lim1 = infantry_queue.size();i<lim1;++i){
            if(infantry_queue[i].target_id == 0){
                infantry_queue[i].target = Eigen::Vector2d(player_infantry.position[0],player_infantry.position[1]);
            }
            if(infantry_queue[i].target_id < 0){
                int id = -infantry_queue[i].target_id-1;
                if(id >= Stone_queue.size()){
                    id = rand()%Stone_queue.size();
                }
                infantry_queue[i].target = Eigen::Vector2d(Stone_queue[id].position[0],Stone_queue[id].position[1]);
            }

            
            Eigen::Vector2d ret = get_global_navigation_velo(
                Eigen::Vector2d(infantry_queue[i].velo[0],infantry_queue[i].velo[1]),
                Eigen::Vector2d(infantry_queue[i].position[0],infantry_queue[i].position[1]),
                infantry_queue[i].target
            );

            infantry_queue[i].velo[1] = ret[1];
            infantry_queue[i].ang_velo = ret[0];

            if((infantry_queue[i].target - Eigen::Vector2d(infantry_queue[i].position[0],infantry_queue[i].position[1])).norm() < 1.0){
                infantry_queue[i].velo[1] = infantry_queue[i].ang_velo = 0;
            }

            if(enemy_able_shoot)
            create_shooting(infantry_queue[i],atan2(infantry_queue[i].target[1]-infantry_queue[i].position[1],infantry_queue[i].target[0]-infantry_queue[i].position[0]));
        }
        
        if(able_shoot && player_infantry.shooter_able)
        create_shooting(player_infantry,player_infantry.shooting_angle);
        return;
    }

    void object_group::start_init(){
        auto WALL_OF_QUEUE = toml::find<std::vector<std::vector<std::vector<double> > > >(toml_file,"WALL_OF_QUEUE");
        auto STONE_OF_QUEUE = toml::find<std::vector<std::vector<double>>>(toml_file,"STONE_OF_QUEUE");
        Eigen::Vector3d end_point_1,end_point_2,position_,norm_;
        Wall element;
        double height_,width_,length_;
        for(auto i:WALL_OF_QUEUE){
            end_point_1[0] = i[0][0];
            end_point_1[1] = i[0][1];
            end_point_2[0] = i[1][0];
            end_point_2[1] = i[1][1];
            
            width_ = toml::find<double>(toml_file,"wall_width");
            height_ = 1.5;
            length_ = (end_point_2 - end_point_1).norm();
            position_ = (end_point_1 + end_point_2)/2;
            norm_ = (end_point_2 - end_point_1).normalized();
            std::swap(norm_[0],norm_[1]);
            norm_[0] = -norm_[0];
            
            element = Wall(position_,norm_,height_,width_,length_,0.3,0.3);
            element.refresh_angle_dot();
            Wall_queue.push_back(element);
        }

        for(auto i:STONE_OF_QUEUE){
            position_[0] = i[0];
            position_[1] = i[1];
            position_[2] = 0;
            infantry element = infantry(position_);
            element.radius = 0.5;
            element.health_point_limit = element.health_point = 2000;
            Stone_queue.push_back(element);
        }

        for(int i=0;i<50;++i){
            for(int j=0;j<50;++j){
                Cuboid item = Cuboid(Eigen::Vector3d(i+0.5,j+0.5,0.),Eigen::Vector3d(1.,0.,0.),1,1,1);

                for(auto k:Wall_queue)
                if(item.judge_inside(k)){
                    Wall_grid[i][j].push_back(k);
                    RCLCPP_INFO(rclcpp::get_logger("judge_inside.."),"%d %d %lf %lf",i,j,k.position[0],k.position[1]);
                }
            }
        }
        return;
    }

    bool object_group::put_soldier(Eigen::Vector3d position){
        infantry element = infantry(position);
        if(element.judge_collision_with_grid(0)){
            return false;
        }
        for(int i=0,lim = infantry_queue.size();i<lim;++i){
            if(element.judge_collision(infantry_queue[i]))
            return false;
        }
        return true;
    }

    object_group::infantry object_group::get_soldier(){
        static unsigned long long mod = (1ull<<32);
        int cnt = 100;
        srand((unsigned)time(NULL));
        S:;
        if(!cnt) return infantry(Eigen::Vector3d(-1,-1,-1));
        --cnt;
        unsigned long long A = 1ull*rand()*rand()*rand()*rand()%mod;
        unsigned long long B = 1ull*rand()*rand()*rand()*rand()%mod;
        double X = 1.*A/mod * map_width_platform;
        double Y = 1.*B/mod * map_height_platform;
        Eigen::Vector3d position = Eigen::Vector3d(X,Y,0.);
        if(!put_soldier(position)) goto S;
        return infantry(position);
    }

    void object_group::refresh(){
        int element_num = 1;
        for(int i=1;i<=element_num;++i){
            infantry soldier = get_soldier();
            soldier.target_id = -(rand()%Stone_queue.size()+1);
            if(soldier.position[0] >= 0)
            infantry_queue.push_back(soldier);
        }
        return;
    }

    bool object_group::segment_cross(std::pair<Eigen::Vector2d,Eigen::Vector2d> segment1,std::pair<Eigen::Vector2d,Eigen::Vector2d> segment2){
        double Ax1,Ax2,Bx1,Bx2,Ay1,Ay2,By1,By2;
        using namespace std;
        Ax1 = segment1.first[0];Ax2 = segment1.second[0];Bx1 = segment2.first[0];Bx2 = segment2.second[0];
        Ay1 = segment1.first[1];Ay2 = segment1.second[1];By1 = segment2.first[1];By2 = segment2.second[1];
        if(
        ( std::max(Ax1,Ax2)>=std::min(Bx1,Bx2)&&std::min(Ax1,Ax2)<=std::max(Bx1,Bx2) )&&  //判断x轴投影
        ( std::max(Ay1,Ay2)>=std::min(By1,By2)&&std::min(Ay1,Ay2)<=std::max(By1,By2) )    //判断y轴投影
        )
        {
            if(
                ( (Bx1-Ax1)*(Ay2-Ay1)-(By1-Ay1)*(Ax2-Ax1) ) *          //判断B是否跨过A
                ( (Bx2-Ax1)*(Ay2-Ay1)-(By2-Ay1)*(Ax2-Ax1) ) <=0 &&
                ( (Ax1-Bx1)*(By2-By1)-(Ay1-By1)*(Bx2-Bx1) ) *          //判断A是否跨过B
                ( (Ax2-Bx1)*(By2-By1)-(Ay2-By1)*(Bx2-Bx1) ) <=0
            )
            {
                return 1;
            }
            else
                return 0;
        }
        else
            return 0;
    }

    void object_group::drawing_field(){
        // RCLCPP_INFO(this->get_logger(),"drawing once! Wall_queue's size:%ld",Wall_queue.size());
        int map_height = map_height_platform * TS_rate,
            map_width = map_width_platform * TS_rate;
        cv::Mat src = cv::Mat(map_height,map_width,CV_8UC3,cv::Scalar(255,255,255));
        std::vector<cv::Point> vec;
        for(int i=0,lim = Wall_queue.size();i<lim;++i){
            vec.clear();
            vec.push_back(cv::Point(Wall_queue[i].angle_dot[0][0]*TS_rate,Wall_queue[i].angle_dot[0][1]*TS_rate));
            vec.push_back(cv::Point(Wall_queue[i].angle_dot[1][0]*TS_rate,Wall_queue[i].angle_dot[1][1]*TS_rate));
            vec.push_back(cv::Point(Wall_queue[i].angle_dot[2][0]*TS_rate,Wall_queue[i].angle_dot[2][1]*TS_rate));
            vec.push_back(cv::Point(Wall_queue[i].angle_dot[3][0]*TS_rate,Wall_queue[i].angle_dot[3][1]*TS_rate));
            // RCLCPP_INFO(this->get_logger(),"draw this,[%lf,%lf],[%lf,%lf],[%lf,%lf],[%lf,%lf]",
            // Wall_queue[i].angle_dot[0][0],Wall_queue[i].angle_dot[0][1],
            // Wall_queue[i].angle_dot[1][0],Wall_queue[i].angle_dot[1][1],
            // Wall_queue[i].angle_dot[2][0],Wall_queue[i].angle_dot[2][1],
            // Wall_queue[i].angle_dot[3][0],Wall_queue[i].angle_dot[3][1]);
            cv::fillPoly(src,vec,cv::Scalar(0,0,0));
        }
        
        for(int i=0,lim = Stone_queue.size();i<lim;++i){
            cv::circle(src,cv::Point(Stone_queue[i].position[0]*TS_rate,Stone_queue[i].position[1]*TS_rate),
                        Stone_queue[i].radius*TS_rate,cv::Scalar(0,255,255),-1);
        }

        static int flag = 0;
        if(!flag){
            cv::imwrite(ROOT "src/check.png",src);
            flag = 1;
        }

        std::string text;
        std::stringstream ss;

        for(int i=0,lim = little_bullet_queue.size();i<lim;++i){
            cv::circle(src,cv::Point(little_bullet_queue[i].position[0]*TS_rate,little_bullet_queue[i].position[1]*TS_rate),
                        little_bullet_queue[i].radius*TS_rate,cv::Scalar(0,0,255),-1);
        }
        for(int i=0,lim = infantry_queue.size();i<lim;++i){
            cv::circle(src,cv::Point(infantry_queue[i].position[0]*TS_rate,infantry_queue[i].position[1]*TS_rate),
                        infantry_queue[i].radius*TS_rate,cv::Scalar(255,0,0),-1);
        }
        cv::circle(src,cv::Point(player_infantry.position[0]*TS_rate,player_infantry.position[1]*TS_rate),player_infantry.radius*TS_rate,cv::Scalar(255,255,0),-1);

        for(int i=0,lim = infantry_queue.size();i<lim;++i){
            ss.clear();text.clear();
            ss<<infantry_queue[i].health_point;
            ss>>text;
            cv::putText(src,text, cv::Point(infantry_queue[i].position[0]*TS_rate, infantry_queue[i].position[1]*TS_rate), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(127, 127, 127), 2);
        }
        for(int i=0,lim = Stone_queue.size();i<lim;++i){
            ss.clear();text.clear();
            ss<<Stone_queue[i].health_point;
            ss>>text;
            cv::putText(src,text, cv::Point(Stone_queue[i].position[0]*TS_rate, Stone_queue[i].position[1]*TS_rate), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(127, 127, 127), 2);
        }

        ss.clear();text.clear();
        ss<<player_infantry.health_point;
        ss>>text;
        cv::putText(src,text, cv::Point(player_infantry.position[0]*TS_rate, player_infantry.position[1]*TS_rate), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(127, 127, 127), 2);

        static std::string window_name = "platform_display";
        cv::imshow(window_name,src);
        cv::waitKey(1);
        return;
    }

    void object_group::timer_callback(){
        publish_detector();
        std::vector<double> pub_pos;pub_pos.resize(2);
        pub_pos[0] = player_infantry.position[0];
        pub_pos[1] = player_infantry.position[1];
        task_msg::msg::Float64MultiArray pub_pos_msg;pub_pos_msg.data = pub_pos;
        pub_pos[0] = player_infantry.velo[0];pub_pos[1] = player_infantry.velo[1];
        task_msg::msg::Float64MultiArray pub_velo_msg;pub_velo_msg.data = pub_pos;
        std_msgs::msg::Float64 pub_ang_velo_msg;pub_ang_velo_msg.data = player_infantry.ang_velo;
        std_msgs::msg::Float64 pub_self_blood_msg;pub_self_blood_msg.data = player_infantry.health_point;
        task_msg::msg::Float64TriMultiArray pub_enemy_msg;pub_enemy_msg.data.resize(enemy_vis_queue.size());
        for(int i=0,lim = enemy_vis_queue.size();i<lim;++i){
            pub_enemy_msg.data[i].x = enemy_vis_queue[i].position[0];
            pub_enemy_msg.data[i].y = enemy_vis_queue[i].position[1];
            pub_enemy_msg.data[i].z = enemy_vis_queue[i].health_point;
        }
        publisher_pos->publish(pub_pos_msg);
        publisher_velo->publish(pub_velo_msg);
        publisher_ang_velo->publish(pub_ang_velo_msg);
        publisher_enemy_info->publish(pub_enemy_msg);
        publisher_self_blood->publish(pub_self_blood_msg);
        static int cnt = 0;++cnt;
        if(cnt>=100)
        cnt=0,player_infantry.add_health_point(1);
        return;
    }

    bool object_group::check_if_able(Eigen::Vector2d start_point,Eigen::Vector2d end_point){
        static double RADIUS = toml::find<double>(toml_file,"car_radius");
        for(int i=0,lim = Wall_queue.size();i<lim;++i){
            double true_length = Wall_queue[i].length/2 + RADIUS;
            double true_width = Wall_queue[i].width/2 + RADIUS;
            Eigen::Vector2d centre = Eigen::Vector2d(Wall_queue[i].position[0],Wall_queue[i].position[1]);
            Eigen::Vector2d vec1 = Eigen::Vector2d(Wall_queue[i].norm[0],Wall_queue[i].norm[1]),
                            vec2 = Eigen::Vector2d(-vec1[1],vec2[0]);
            Eigen::Vector2d dot1 = centre + vec1 * true_width + vec2 * true_length,
                            dot2 = centre + vec1 * true_width - vec2 * true_length,
                            dot3 = centre - vec1 * true_width - vec2 * true_length,
                            dot4 = centre - vec1 * true_width + vec2 * true_length;
            // printf("[[%lf,%lf],[%lf,%lf]]---[[%lf,%lf],[%lf,%lf]]:%d\n",
            // start_point[0],start_point[1],end_point[0],end_point[1],
            // dot1[0],dot1[1],dot2[0],dot2[1],segment_cross(std::make_pair(start_point,end_point),
            //                 std::make_pair( Eigen::Vector2d(dot1[0],dot1[1]),
            //                                 Eigen::Vector2d(dot2[0],dot2[1]))));
            if(segment_cross(std::make_pair(start_point,end_point),std::make_pair(dot1,dot2)))   return false;
            if(segment_cross(std::make_pair(start_point,end_point),std::make_pair(dot2,dot3)))   return false;
            if(segment_cross(std::make_pair(start_point,end_point),std::make_pair(dot3,dot4)))   return false;
            if(segment_cross(std::make_pair(start_point,end_point),std::make_pair(dot4,dot1)))   return false;
        }

        for(int i=0,lim = Stone_queue.size();i<lim;++i){
            Eigen::Vector2d dot1 = (end_point-Eigen::Vector2d(Stone_queue[i].position[0],Stone_queue[i].position[1]));
            Eigen::Vector2d dot2 = (start_point-Eigen::Vector2d(Stone_queue[i].position[0],Stone_queue[i].position[1]));
            Eigen::Vector2d dot3 = (start_point-end_point);
            std::swap(dot3[0],dot3[1]);dot3[0] = -dot3[0];

            double dis = std::min(dot1.norm(),dot2.norm());

            if((dot3[0]*dot1[1]-dot3[1]*dot1[0])*(dot3[0]*dot2[1]-dot3[1]*dot2[0])<=0){
                double dis1 = ((dot1+dot2)/2).dot(dot3) / dot3.norm();
                dis1 = fabs(dis1);
                // RCLCPP_INFO(rclcpp::get_logger("judge_collision_once"),"%d %lf %lf",i,dis1,dist_limit);
                if(dis1 < dis) dis = dis1;
            }

            if(dot2.norm() < Stone_queue[i].radius) continue;

            if(dis < RADIUS + Stone_queue[i].radius)
            return false;
        }
        return true;
    }

    void object_group::publish_detector(){
        enemy_vis_queue.clear();
        Eigen::Vector2d start_point = Eigen::Vector2d(player_infantry.position[0],player_infantry.position[1]);
        for(int i=0,lim = infantry_queue.size();i<lim;++i){
            if(check_if_able(start_point,Eigen::Vector2d(infantry_queue[i].position[0],infantry_queue[i].position[1])))
            enemy_vis_queue.push_back(infantry_queue[i]);
        }
        return;
    }

    void object_group::server_locator(const std::shared_ptr<task_msg::srv::NavFind::Request> request,std::shared_ptr<task_msg::srv::NavFind::Response>      response){
        // RCLCPP_INFO(this->get_logger(),"here locate!!!!");
        // for(int i=0,lim = request->pos_post.data.size();i<lim;++i){
        //     printf("%lf %lf\n",request->pos_post.data[i].x,request->pos_post.data[i].y);
        // }
        // printf("%lf %lf\n",request->pos.data[0],request->pos.data[1]);
        task_msg::msg::Float64PairMultiArray request_pos_array = request->pos_post;
        Eigen::Vector2d request_pos = Eigen::Map<Eigen::Vector2d>(request->pos.data.data());
        std::vector<Eigen::Vector2d> position_array;position_array.resize(request_pos_array.data.size());
        for(int i=0,lim = request_pos_array.data.size();i<lim;++i)
        position_array[i] = Eigen::Vector2d(request_pos_array.data[i].x,request_pos_array.data[i].y);
        for(int i=0,lim = position_array.size();i<lim;++i){
            // printf("%lf %lf %lf %lf\n",request_pos[0],request_pos[1],position_array[i][0],position_array[i][1]);
            if(check_if_able(request_pos,position_array[i]))
            response->id.push_back(i);//,printf("%d %lf %lf\n",i,position_array[i][0],position_array[i][1]);
        }
        return;
    }

    Eigen::Vector2d object_group::get_global_navigation_velo(Eigen::Vector2d old_velo,Eigen::Vector2d start_point,Eigen::Vector2d end_point){
        // RCLCPP_INFO(this->get_logger(),"HAVEN'T!");
        auto request = std::make_shared<task_msg::srv::PathPlan::Request>();
        request->old_velo.data = std::vector<double>{old_velo[0],old_velo[1]};
        request->start_point.data = std::vector<double>{start_point[0],start_point[1]};
        request->end_point.data = std::vector<double>{end_point[0],end_point[1]};
        while (!client_nav_global->wait_for_service(std::chrono::milliseconds(1000))){RCLCPP_INFO(this->get_logger(),"I'm waiting....");}
        auto result = client_nav_global->async_send_request(request);
        rclcpp::spin_until_future_complete(node_nav_global,result);
        // RCLCPP_INFO(this->get_logger(),"HAS!");
        return Eigen::Map<Eigen::Vector2d>(result.get()->velo.data.data());
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(platform::object_group)