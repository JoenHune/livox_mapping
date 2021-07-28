//
// Created by joen on 2021/7/16.
//

#include "lidarOdometry.h"

LidarOdometry::LidarOdometry(ros::NodeHandle &nh) {
    _pub_localMap = nh.advertise<sensor_msgs::PointCloud2>(
            "/lidar/local_map", 20000);
    _pub_newScan = nh.advertise<sensor_msgs::PointCloud2>(
            "/lidar/scan", 20000);

    _sub_scanMsg = nh.subscribe<sensor_msgs::PointCloud2>(
            "/livox/lidar", 10000, &LidarOdometry::scanHandler, this);

    _scan_in.reset(new pcl::PointCloud<PointType>());
    _scan_reserved.reset(new pcl::PointCloud<PointType>());
    _local_map.reset(new pcl::PointCloud<PointType>());

    _ikd_tree = new KD_TREE(0.3, 0.6, 0.2);

    // initial parameters
    _undistort = false;
    _distance_threshold = 6;
    _time_between_2frames = chrono::system_clock::now();

    for (int i = 0; i < int(TOTAL_RELATIONS); i++) {
        X.emplace_back(
                std::make_pair<Eigen::Quaternionf, Eigen::Vector3f>(Eigen::Quaternionf(1, 0, 0, 0),
                                                                    Eigen::Vector3f(0, 0, 0)));
    }

    ROS_INFO("Prepare to propose new scan, initial anchor: q=(%f, %f, %f, %f), t=(%f, %f, %f)",
             X[WORLD_TO_LAST].first.w(),
             X[WORLD_TO_LAST].first.x(),
             X[WORLD_TO_LAST].first.y(),
             X[WORLD_TO_LAST].first.z(),
             X[WORLD_TO_LAST].second.x(),
             X[WORLD_TO_LAST].second.y(),
             X[WORLD_TO_LAST].second.z());
}

LidarOdometry::~LidarOdometry() {
    delete _ikd_tree;
    _ikd_tree = nullptr;
}

void LidarOdometry::scanHandler(const sensor_msgs::PointCloud2ConstPtr &scanMsg) {
    std::vector<chrono::time_point<chrono::system_clock>> t(10, chrono::system_clock::now());
    resetParameters();

    // Put in slide window
    _slide_window.push_back(scanMsg);
    // Marginalization: Naive version
    if (_slide_window.size() > 100) {
        _slide_window.erase(_slide_window.cbegin());
    }
    t[1] = chrono::system_clock::now();
    ROS_INFO("==> Copy and clean, with now num of pts in tree: %d", _ikd_tree->size());

    // Copy and remove NAN points from ROS Message
    pcl::fromROSMsg(*scanMsg, *_scan_in);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_scan_in, *_scan_in, indices);

    t[2] = chrono::system_clock::now();

    // add left points from last scan (if have)
    if (!_scan_reserved->empty())
        *_scan_in += *_scan_reserved;
    t[3] = chrono::system_clock::now();

    // Registration
    if (_slide_window.size() < 20) {
        t[4] = chrono::system_clock::now();
        *_local_map += *_scan_in;
        if (_ikd_tree->size() == 0) {
            _ikd_tree->Build(toPointVector(_scan_in));
        } else {
            _ikd_tree->Add_Points(toPointVector(_scan_in), true);
        }
        t[5] = chrono::system_clock::now();
    } else {
        bool converged;
        Eigen::Matrix4f T_k1_k;
        if (_slide_window.size() == 20)
            converged = transformScanToLocalMap2(T_k1_k);
        else {
            converged = transformScanToLocalMap(T_k1_k);
        }
        t[4] = chrono::system_clock::now();

        // Check if has converged
        if (converged) {
            // state management
            X[WORLD_TO_CURRENT] = toState(T_k1_k);
            updateTransformations();

            // local map management
            *_local_map += *_scan_in;
            _ikd_tree->Add_Points(toPointVector(_scan_in), true);

            // empty reserved points
            _scan_reserved->clear();
        } else {
            // 继续预测一次
            pcl::transformPointCloud(*_scan_in, *_scan_in, toMatrix(X[LAST_TO_CURRENT]));
            _scan_reserved.swap(_scan_in);
        }
        t[5] = chrono::system_clock::now();
    }

//    ROS_INFO("==> Publish");
    publishPointClouds(scanMsg->header);
    t[6] = chrono::system_clock::now();

    ROS_INFO("Time status");
    ROS_INFO("===========");
    ROS_INFO("1. slide window:                      \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[1] - t[0]).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    ROS_INFO("2. copy and clean:                    \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[2] - t[1]).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    ROS_INFO("3. add points left before:            \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[3] - t[2]).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    ROS_INFO("4. transformation between two frames: \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[4] - t[3]).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    ROS_INFO("5. local map and kd-tree maintenance: \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[5] - t[4]).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    ROS_INFO("6. pointclouds publishing:            \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[6] - t[5]).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    ROS_INFO("7. total time costing:                \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[6] - t[0]).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    ROS_INFO("8. from last frame:                   \t %f ms",
             double(chrono::duration_cast<chrono::microseconds>(t[6] - _time_between_2frames).count()) *
                     chrono::microseconds::period::num / chrono::microseconds::period::den );
    _time_between_2frames = chrono::system_clock::now();
}

std::vector<PointType> LidarOdometry::toPointVector(const pcl::PointCloud<PointType>::Ptr& pointcloud) {
    std::vector<PointType> result;
    for (auto p : pointcloud->points)
        result.push_back(p);
    return result;
}

StateType LidarOdometry::toState(const Eigen::Matrix4f &transform) {
    return {std::make_pair<Eigen::Quaternionf, Eigen::Vector3f>(
            Eigen::Quaternionf(transform.block<3, 3>(0, 0)),
            Eigen::Vector3f(transform.col(3).head(3))
            )};
}

Eigen::Matrix4f LidarOdometry::toMatrix(const StateType &state) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = state.first.toRotationMatrix();
    transform.col(3).head(3) = state.second;
    return transform;
}

void LidarOdometry::resetParameters() {

}

void LidarOdometry::updateTransformations() {
    // 调用时机：新一帧的位姿被优化完后

    // 计算帧间位姿变换
    // △q = q_k * q_{k-1}.inv
    // △t = t_k - △q t_{k-1}
    X[LAST_TO_CURRENT].first = X[WORLD_TO_CURRENT].first * X[WORLD_TO_LAST].first.inverse();
    X[LAST_TO_CURRENT].second = X[WORLD_TO_CURRENT].second - X[LAST_TO_CURRENT].first * X[WORLD_TO_LAST].second;

    // 预测新的位姿
    // q_{k+1} = △q * q_k
    // t_{k+1} = △t + △q t_k
    X[WORLD_TO_PREDICT].first = X[LAST_TO_CURRENT].first * X[WORLD_TO_CURRENT].first;
    X[WORLD_TO_PREDICT].second = X[LAST_TO_CURRENT].second + X[LAST_TO_CURRENT].first * X[WORLD_TO_CURRENT].second;

    // 记录新的位姿
    X[WORLD_TO_LAST] = X[WORLD_TO_CURRENT];
    _trajectory.push_back(X[WORLD_TO_CURRENT]);
}

bool LidarOdometry::transformScanToLocalMap(Eigen::Matrix4f &transform) {

    // 矫正运动畸变
    if (_undistort) {
        pcl::PointCloud<PointType> undistorted(*_scan_in);
        auto _size = undistorted.size();
        for (int i = 0; i < _size; i++) {
            auto R = X[WORLD_TO_CURRENT].first.slerp(float(i) / float(_size), X[WORLD_TO_PREDICT].first);
            auto t = float(i) / 1000.f * X[LAST_TO_CURRENT].second;
            auto p = (R * undistorted.points[i].getVector3fMap() + t);
            undistorted.points[i] = {p.x(), p.y(), p.z()};
        }
        *_scan_in = undistorted;
    }

    // 尝试将点云变换回全局坐标系
    pcl::PointCloud<PointType>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*_scan_in, *cloud, toMatrix(X[WORLD_TO_PREDICT]).inverse());

    int K = 5, k;
    int num_features = 0;
    std::vector<PointType> surroundings;
    std::vector<float> distances;
    std::vector<PointType>::iterator it;

    // 待优化变量
    double param_q[4] = {1, 0, 0, 0};
    double param_t[3] = {0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> result_q(param_q);
    Eigen::Map<Eigen::Vector3d> result_t(param_t);

    // 构造 ceres 优化问题
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(param_q, 4, q_parameterization);
    problem.AddParameterBlock(param_t, 3);
    // 叠加所有点
    for (const auto& p : *cloud) {
        // k-NN 寻找最近的平面 patch
        _ikd_tree->Nearest_Search(p, K, surroundings, distances);

        // 去除距离过于远的点，但如果剩余点不足3，则跳过当前的参考点
        for (k = 0, it = surroundings.begin(); k < distances.size(); k++, it++)
            if (distances[k] > _distance_threshold) {
//                ROS_WARN("%d-th surrounding point (%f, %f, %f) is too far away, distance =  %f",
//                         k, (*it).x, (*it).y, (*it).z, distances[k]);
                surroundings.erase(it);
            }
        if (surroundings.size() < 3) continue;

        auto *cost_function = LidarPlaneFactor::Create(p.getVector3fMap(), surroundings);
        problem.AddResidualBlock(cost_function, loss_function, param_q, param_t);
        num_features++;
    }
    ROS_WARN("Number of points: %d", num_features);

    if (num_features <= 0) return false;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 30;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_INFO("Optimization finished with %zu loops, summary: \n %s",
             summary.iterations.size(), summary.BriefReport().c_str());

    auto termination = summary.termination_type;
    if (!summary.iterations.empty() && termination == ceres::CONVERGENCE) {
        transform.block<3, 3>(0, 0) = result_q.cast<float>().toRotationMatrix();
        transform.col(3).head(3) = result_t.cast<float>();
        return true;
    } else return false;
}

bool LidarOdometry::transformScanToLocalMap2(Eigen::Matrix4f &transform) {
    ROS_INFO("Figure out transform, by using ICP algorithm");
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaximumIterations(20);
    icp.setInputSource(_scan_in);
    icp.setInputTarget(_local_map);
    ROS_INFO("_transform_list.size() = %zu", _trajectory.size());
    icp.align(*_scan_in, _trajectory.empty() ? Eigen::Matrix4f::Identity()
                                                      : toMatrix(_trajectory.back()));

    if (icp.hasConverged()) {
        transform = icp.getFinalTransformation();
        return true;
    } else {
        ROS_ERROR("ICP has not converged, reserve scan.");
        transform = Eigen::Matrix4f::Identity();
        return false;
    }
}

void LidarOdometry::publishPointClouds(std_msgs::Header header) {

    sensor_msgs::PointCloud2 cloudTemp;

    auto PublishCloud = [&](ros::Publisher &pub, const pcl::PointCloud<PointType>::Ptr &cloud) {
        if (pub.getNumSubscribers() != 0) {
            pcl::toROSMsg(*cloud, cloudTemp);
            cloudTemp.header.stamp = header.stamp;
            cloudTemp.header.frame_id = "livox_frame";
            pub.publish(cloudTemp);
        }
    };

    PublishCloud(_pub_localMap, _local_map);
    PublishCloud(_pub_newScan, _scan_in);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle nh;

    LidarOdometry odom(nh);

    ros::MultiThreadedSpinner spinner(16);
    ros::spin(spinner);

    return 0;
}