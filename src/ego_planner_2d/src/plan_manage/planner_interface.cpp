#include <plan_manage/planner_interface.h>

#include <algorithm>
#include <cmath>

namespace ego_planner
{

    PlannerInterface::PlannerInterface()
    {

    }

    PlannerInterface::~PlannerInterface()
    {

    }

    void PlannerInterface::initParam(double max_vel,double max_acc,double max_jerk)
    {
        pp_.max_vel_ = max_vel;
        pp_.max_acc_ = max_acc;
        pp_.max_jerk_ = max_jerk;
        pp_.feasibility_tolerance_ = 0.05;
        pp_.ctrl_pt_dist = 0.4;
        pp_.planning_horizen_ = 5.0;
    }
    
    void PlannerInterface::initEsdfMap(double x_size,double y_size,double resolution,double inflate_values)
    {
        std::cout << "x_size =" << x_size << " y_size =" << y_size << std::endl;
        std::cout << "resolution =" << resolution << std::endl;
        std::cout << "inflate_values =" << inflate_values << std::endl;
        //初始化gridmap地图
        Eigen::Vector2i map_size(x_size,y_size);     
        grid_map_     = std::make_shared<GridMap2D>(resolution,map_size);
        grid_map_->setInflateRadius(inflate_values);

        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam();
        bspline_optimizer_rebound_->setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        bspline_optimizer_rebound_->a_star_.reset(new AStar);
        // A* 的栅格池是"格数"，空间跨度 = 格数 × step_size。step_size 现在跟随
        // grid_map_->getResolution()（见 makePlan 和 bspline_optimizer 里的
        // AstarSearch 调用），所以池子必须按分辨率算：
        //     原来 100 格 × 0.10 m = 10 m 跨度
        //     0.05 m 分辨率下若还是 100 格，只有 5 m —— 场地对角线 5.7 m，
        //     绕一次障碍就会打印 "Ran out of pool" 并让这一段没有 A* 兜底
        // 所以按"10 m 跨度"反算格数，分辨率变了跨度不变。
        const int astar_pool =
            std::max(16, static_cast<int>(std::lround(10.0 / resolution)));
        bspline_optimizer_rebound_->a_star_->initGridMap(
            grid_map_, Eigen::Vector2i(astar_pool, astar_pool));
        std::cout << "a_star pool = " << astar_pool << " x " << astar_pool
                  << " cells at " << resolution << " m = "
                  << astar_pool * resolution << " m span" << std::endl;
       
    }

    void PlannerInterface::setPathPoint(std::vector<PathPoint> &plan_traj)
    {
        _global_plan_traj_.clear();
        _global_plan_traj_ = plan_traj;
    }
    
    void PlannerInterface::setObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        for(int i = 0; i < obstacle.size();i++)
        {
            Eigen::Vector2d coord(obstacle[i].x,obstacle[i].y);
            Eigen::Vector2i res;
            res = grid_map_->worldToGrid(coord);
            grid_map_->setObstacle(res, true);
        }

        grid_map_->inflate();

        // for(int i = 0; i < obstacle.size();i++)
        // {
        //     Eigen::Vector3d pos;
        //     pos[0] = obstacle[i].x;
        //     pos[1] = obstacle[i].y;
        //     pos[2] = 1;
        //     grid_map_->getInflateOccupancy(pos);
        //     std::cout << "grid_map_->getInflateOccupancy(pos); =pos " << pos << " ," << grid_map_->getInflateOccupancy(pos) << std::endl;
        // }
       

    }

    void PlannerInterface::setCurrentVehiclePos(PathPoint& cur_pose)
    {
        cur_pose_ = cur_pose;
        // grid_map_->resetGrids();
    }  

    void PlannerInterface::setGridMap(PathPoint& cur_pose)
    {
        grid_map_->setCurPose(cur_pose.x,cur_pose.y); // recenters the local map on the vehicle
    }

    void PlannerInterface::resetMap()
    {
        grid_map_->resetMap();
    }

    bool PlannerInterface::getInflateOccupancy(PathPoint& traj)
    {
        Eigen::Vector2d pos;
        pos[0] = traj.x;
        pos[1] = traj.y;
        return grid_map_->getInflateOccupancy(pos);
    }

    void PlannerInterface::getObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        std::vector<Eigen::Vector2d> inflated_cloud = grid_map_->getObstaclePointCloud();

        for (const auto& point : inflated_cloud)
        {
            ObstacleInfo temp;
            temp.x = static_cast<float>(point.x()); // 解析 x
            temp.y = static_cast<float>(point.y()); // 解析 y
            obstacle.push_back(temp);
        }



        //         // 1. 获取膨胀后的障碍物点云（默认）
        // std::vector<Eigen::Vector2d> inflated_cloud = grid_map.getObstaclePointCloud();

        // // 2. 获取原始障碍物点云
        // std::vector<Eigen::Vector2d> raw_cloud = grid_map.getObstaclePointCloud(false);
    }


    void PlannerInterface::makePlan()
    {
        _plan_traj_results_.clear();
        a_star_pathes_.clear();
        std::cout << "开始规划..." << std::endl;
        Eigen::Vector3d start_pt;
        Eigen::Vector3d start_vel;
        Eigen::Vector3d start_acc;
        Eigen::Vector3d local_target_pt;
        Eigen::Vector3d local_target_vel;
        vector<Eigen::Vector3d> point_set;


        vector<Eigen::Vector3d> traj_pts;

        for(int i = 0; i< _global_plan_traj_.size();i++)
        {
            Eigen::Vector3d plan_pt(_global_plan_traj_[i].x,_global_plan_traj_[i].y,0.2);
            point_set.push_back(plan_pt);
        }

        if (point_set.size() == 2)
        {
            const Eigen::Vector3d midpoint =
                0.5 * (point_set.front() + point_set.back());
            point_set.insert(point_set.begin() + 1, midpoint);
        }

        if(point_set.size() < 3)
        {
            printf("Global path has too few points (%zu), skip optimization.\n", point_set.size());
            return;
        }

        start_pt[0] = cur_pose_.x;
        start_pt[1] = cur_pose_.y;
        start_pt[2] = 0.0;
        
        local_target_pt[0] = _global_plan_traj_[_global_plan_traj_.size()-1].x;
        local_target_pt[1] = _global_plan_traj_[_global_plan_traj_.size()-1].y;
        local_target_pt[2] = 0;

        // Mission segments begin from a position hold; a unit-speed
        // derivative would exceed the configured max_vel_ of 0.35 m/s.
        start_vel.setZero();

        local_target_vel[0] = 0;//根据实际需求修改接入
        local_target_vel[1] = 0;
        local_target_vel[2] = 0;

        start_acc.setZero();


        auto start = std::chrono::system_clock::now();

        // ------------------------------------------------------------------
        // The B-spline optimizer and the A* fallback do NOT accept the same
        // input sizes, so the dispatch has to happen here (2026-09-25).
        //
        // UniformBspline::parameterizeToBspline() builds A as (K+4) x (K+2),
        // so a K-point reference path yields K+2 control points, and
        // BsplineOptimizer::rebound_optimize() needs
        //     variable_num_ = 2 * ((K + 2) - 2 * order_) > 0,  order_ = 3
        // i.e. K >= 5. Handing it a 4-point path produced variable_num_ = 0,
        // lbfgs returned LBFGSERR_INVALID_N (-1021) and printed
        // "Invalid number of variables specified.", and the plan was silently
        // dropped - which is the freeze this fix removes. A short path is not
        // an error, it just belongs to the A* branch, which needs only 2 points.
        // ------------------------------------------------------------------
        const size_t kMinPointsForBspline = 5;
        bool plan_success = false;
        if (point_set.size() >= kMinPointsForBspline)
        {
            plan_success = reboundReplan(start_pt, start_vel, start_acc,
                                         local_target_pt, local_target_vel, point_set);
        }
        else
        {
            printf("Reference path has only %zu point(s); the B-spline optimizer "
                   "needs at least %zu. Dispatching straight to the A* fallback.\n",
                   point_set.size(), kMinPointsForBspline);
        }

        if (plan_success)
        {
            getTraj();
        }
        else
        {
            const Eigen::Vector2d astar_start(start_pt.x(), start_pt.y());
            const Eigen::Vector2d astar_goal(local_target_pt.x(), local_target_pt.y());
            const double start_to_goal = (astar_goal - astar_start).norm();
            // step_size 必须等于地图分辨率：Index2Coord() 用
            // (index - CENTER_IDX) * step_size + center_ 反算世界坐标，step 与
            // grid_map_ 的格子不一致时，"A* 认为自由的格"和"地图认为自由的格"
            // 就是两套坐标，0.05 m 的地图上按 0.10 搜等于只查了一半的格子。
            const double astar_step = grid_map_->getResolution();
            if (start_to_goal > 0.1 &&
                bspline_optimizer_rebound_->a_star_->AstarSearch(
                    astar_step, astar_start, astar_goal))
            {
                const std::vector<Eigen::Vector2d> fallback =
                    bspline_optimizer_rebound_->a_star_->getPath();
                if (fallback.size() >= 2)
                {
                    // AStar::ConvertToIndexAndAdjustStartEndPoints() walks the
                    // endpoint BACK toward the start whenever the goal itself
                    // lands inside an inflated cell, so a perfectly good path
                    // legitimately ends short of astar_goal. The old test
                    // required (fallback.back() - astar_goal).norm() <= 0.15 and
                    // therefore threw away every path whose goal was inflated -
                    // exactly the case a fallback exists for. Accept strict
                    // progress toward the goal as well.
                    const double end_to_goal = (fallback.back() - astar_goal).norm();
                    const bool reaches_goal = end_to_goal <= 0.15;
                    const bool makes_progress = end_to_goal < start_to_goal;
                    if (reaches_goal || makes_progress)
                    {
                        _plan_traj_results_.reserve(fallback.size());
                        for (const Eigen::Vector2d &point : fallback)
                        {
                            PathPoint result;
                            result.x = static_cast<float>(point.x());
                            result.y = static_cast<float>(point.y());
                            result.z = 0.0f;
                            result.v = 0.0f;
                            _plan_traj_results_.push_back(result);
                        }
                        std::cout << "Using collision-checked A* fallback with "
                                  << fallback.size() << " points ("
                                  << (reaches_goal ? "reaches the goal" : "makes progress")
                                  << ", end " << end_to_goal << " m from the goal)."
                                  << std::endl;
                    }
                    else
                    {
                        printf("A* fallback ends %.3f m from the goal, no better than "
                               "the current %.3f m; discarding.\n",
                               end_to_goal, start_to_goal);
                    }
                }
            }
        }
           
        auto end = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        printf("MotionPlanner Total Running Time: %ld  ms \n", elapsed.count());
    }

    void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results)
    {
        plan_traj_results = _plan_traj_results_;
    }
    
    void PlannerInterface::getAStarPath(vector<vector<Eigen::Vector2d>>& a_star_path)
    {
        a_star_path = a_star_pathes_;

    }


    bool PlannerInterface::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set)
    {
        vector<Eigen::Vector3d> start_end_derivatives;
        double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(start_acc);

        Eigen::MatrixXd ctrl_pts_3d;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts_3d);

        // parameterizeToBspline() bails out silently (printing "[B-spline]:point
        // set have only N points.") when point_set.size() <= 3 and leaves
        // ctrl_pts_3d empty, so topRows(2) below would be an out-of-range
        // Eigen operation. Guard it here as well - makePlan() should already
        // have routed those paths to the A* fallback.
        const int bspline_order = bspline_optimizer_rebound_->getOrder();
        if (ctrl_pts_3d.cols() <= 2 * bspline_order)
        {
            printf("B-spline parameterization produced only %d control point(s) "
                   "(need > %d); skipping the optimizer.\n",
                   static_cast<int>(ctrl_pts_3d.cols()), 2 * bspline_order);
            return false;
        }

        Eigen::MatrixXd ctrl_pts_2d;

        ctrl_pts_2d = ctrl_pts_3d.topRows(2);

        a_star_pathes_ = bspline_optimizer_rebound_->initControlPoints(ctrl_pts_2d, true);
        
        static int vis_id = 0;
        
        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts_2d, ts);
        cout << "first_optimize_step_success flag=" << flag_step_1_success << endl;
        if (!flag_step_1_success)
        {
            std::cout << "轨迹优化器优化失败！"  << std::endl;
            continous_failures_count_++;
            return false;
        }
        else 
        {
            std::cout << "轨迹优化器优化成功！"  << std::endl;
        }

        ctrl_pts_3d = bspline_optimizer_rebound_->convert2DTo3D(ctrl_pts_2d);
        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts_3d, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;
        if (!pos.checkFeasibility(ratio, false))
        {
            cout << "Need to reallocate time." << endl;

            Eigen::MatrixXd optimal_control_points;
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
            if (flag_step_2_success)
                pos = UniformBspline(optimal_control_points, 3, ts);
        }

        if (!flag_step_2_success)
        {
            printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            continous_failures_count_++;
            // return false;
        }
    
        updateTrajInfo(pos);

        continous_failures_count_ = 0;

        return true;
    }

    bool PlannerInterface::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {

        Eigen::MatrixXd optimal_control_point_temp;
        double t_inc;

        Eigen::MatrixXd ctrl_pts; 

        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        Eigen::MatrixXd ctrl_pts_2d;

        ctrl_pts_2d = ctrl_pts.topRows(2);

        traj = UniformBspline(ctrl_pts, 3, ts);

        

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);

        bspline_optimizer_rebound_->ref_pts_.clear();

        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t).topRows(2));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts_2d, ts, optimal_control_point_temp);

        optimal_control_points = bspline_optimizer_rebound_->convert2DTo3D(optimal_control_point_temp);
         
        return success;
    }

    void PlannerInterface::updateTrajInfo(const UniformBspline &position_traj)
    {
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void PlannerInterface::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                            Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }

        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

    void PlannerInterface::getTraj()
    {
        auto info = &local_data_;

        ego_planner::Bspline bspline;
        bspline.order = 3;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());

        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());

        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        vector<ego_planner::UniformBspline> traj_;
        double traj_duration_;

        ego_planner::UniformBspline pos_traj(pos_pts, bspline.order, 0.1);
        pos_traj.setKnot(knots);
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());
        traj_.push_back(traj_[1].getDerivative());
        traj_duration_ = traj_[0].getTimeSum();


        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
        _plan_traj_results_.clear();
        for (double t_cur = 0; t_cur <= traj_duration_; t_cur += 0.1) 
        {
            pos = traj_[0].evaluateDeBoorT(t_cur);
            vel = traj_[1].evaluateDeBoorT(t_cur);
            acc = traj_[2].evaluateDeBoorT(t_cur);
            PathPoint tempPath;
            tempPath.x = pos(0);
            tempPath.y = pos(1);
            _plan_traj_results_.push_back(tempPath);
        }
    }

}
