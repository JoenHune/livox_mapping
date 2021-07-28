//
// Created by joen on 2021/7/23.
//

#include "Eigen/Dense"
#include "ceres/ceres.h"
#include <utility>
#include <vector>
#include "misc.h"

// 计算Odometry线程中点到面的残差距离
struct LidarPlaneFactor
{
    LidarPlaneFactor(const Eigen::Vector3f& current_point,
                     const std::vector<PointType>& surroundings)
            : p_j(current_point.cast<double>())
    {
        // 平面拟合：求重心 q_j
        q_j.setZero();
        for (const auto& neighbor : surroundings) {
            q_j += neighbor.getVector3fMap().cast<double>();
        }
        q_j /= double(surroundings.size());

        // 平面拟合：求法向量 u_j
        Eigen::Matrix3d A;
        A.setZero();
        for (const auto& neighbor : surroundings) {
            n_j = neighbor.getVector3fMap().cast<double>();
            n_j -= q_j;
            A += (n_j * n_j.transpose()); // n_j 默认为列向量
        }
        Eigen::EigenSolver<Eigen::Matrix3d> svd(A);
        Eigen::MatrixXd::Index idx;
        svd.eigenvalues().real().rowwise().squaredNorm().minCoeff(&idx);
        u_j = svd.eigenvectors().real().col(idx);
    }

    template <typename T>
    bool operator()(const T *param_q, const T *param_t, T *residual) const
    {
        Eigen::Quaternion<T> quaternion {param_q[3], param_q[0], param_q[1], param_q[2]};
        Eigen::Matrix<T, 3, 1> t {param_q[0], param_t[1], param_t[2]};
        Eigen::Matrix<T, 3, 1> u {T(u_j[0]), T(u_j[1]), T(u_j[2])};
        Eigen::Matrix<T, 3, 1> p {T(p_j[0]), T(p_j[1]), T(p_j[2])};
        Eigen::Matrix<T, 3, 1> q {T(q_j[0]), T(q_j[1]), T(q_j[2])};

        // 计算点到平面的残差距离，如下图所示
        auto x = quaternion * p + t - q;
        auto d = x.transpose() * u * u.transpose() * x;
        residual[0] = T(d);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3f& current_point,
                                       const std::vector<PointType>& surroundings)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarPlaneFactor, 1, 4, 3>(
//				 	              ^  ^  ^
//			         残差的维度 ____|  |  |
//			   优化变量 q 的维度 _______|  |
//		 	   优化变量 t 的维度 __________|
                new LidarPlaneFactor(current_point, surroundings)));
    }

    Eigen::Vector3d p_j, q_j, u_j, n_j;
};