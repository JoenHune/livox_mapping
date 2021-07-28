//
// Created by joen on 2021/7/16.
//

#ifndef LIVOX_MAPPING_LIDARODOMETRY_H
#define LIVOX_MAPPING_LIDARODOMETRY_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"
#include "misc.h"
#include <vector>

#include "Eigen/Dense"

#include "ceres/ceres.h"

#include "lidarFactor.h"
#include "ikd_Tree.h"
#include <chrono>

/* TODO:
 *  1. 初始化阶段需要一定时间静置，这段时间认为雷达是不动的，初始化位姿 Anchor 为 q_w_0(1, 0, 0, 0), t_w_0(0, 0, 0)，
 *     将点云直接作为 local map
 *  2. 初始化完成后的第一帧直接 ICP 求解位姿变换，得到 q_w_1 , t_w_1 ，此时可以反推得到速度 △q_w = q_w_1.inverse() * q_w_0
 *     和位移 △t_w = t_w_1 - t_w_0
 *  3. 依据速度 △q_w 和位移 △t_w 预测新一帧的 q_w_k, t_w_k ，对新一帧的点云做变换使其落到世界坐标系上
 *  4. 使用观测约束优化 q_w_k, t_w_k 使误差最小
 *  5. 将点云变换到世界坐标系上，加入到局部地图中
 *  6. 局部地图的边缘化后续再考虑
 */

class LidarOdometry {
public:
    explicit LidarOdometry(ros::NodeHandle& nh);

    ~LidarOdometry();

    void scanHandler(const sensor_msgs::PointCloud2ConstPtr &scanMsg);

private:

    std::vector< sensor_msgs::PointCloud2ConstPtr > _slide_window;

    pcl::PointCloud<PointType>::Ptr _scan_in;
    pcl::PointCloud<PointType>::Ptr _scan_reserved;
    pcl::PointCloud<PointType>::Ptr _local_map;

    chrono::time_point<chrono::system_clock> _time_between_2frames;

    /// kd-tree
    KD_TREE* _ikd_tree;

    float _distance_threshold;

    /// states
    /// ======

    typedef enum {
        LAST_TO_CURRENT = 0,   // [translation] (k-1)-th frame to k-th frame
        WORLD_TO_LAST = 1,     // [world frame] last ((k-1)-th) frame to world frame
        WORLD_TO_CURRENT = 2,  // [world frame] current (k-th) frame to world frame
        WORLD_TO_PREDICT = 3,  // [world frame] predict ((k+1)-th) frame to world frame
        TOTAL_RELATIONS
    } frame_relation;

    std::vector< StateType > X;
    std::vector< StateType > _trajectory;

    bool _undistort;

    /// publishers and subscribers
    /// ==========================
    // publishers
    ros::Publisher _pub_localMap;
    ros::Publisher _pub_newScan;

    // subscribers
    ros::Subscriber _sub_scanMsg;

    void resetParameters();
    void publishPointClouds(std_msgs::Header header);

    void updateTransformations();

    bool transformScanToLocalMap(Eigen::Matrix4f &transform);
    bool transformScanToLocalMap2(Eigen::Matrix4f &transform);

    static std::vector<PointType> toPointVector(const pcl::PointCloud<PointType>::Ptr& pointcloud);
    static std::pair<Eigen::Quaternionf, Eigen::Vector3f> toState(const Eigen::Matrix4f &transform);
    static Eigen::Matrix4f toMatrix(const std::pair<Eigen::Quaternionf, Eigen::Vector3f> &transform);
};

#endif //LIVOX_MAPPING_LIDARODOMETRY_H
