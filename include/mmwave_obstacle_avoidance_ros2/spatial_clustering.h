#ifndef MMWAVE_OBSTACLE_AVOIDANCE_ROS2_SPATIAL_CLUSTERING_H
#define MMWAVE_OBSTACLE_AVOIDANCE_ROS2_SPATIAL_CLUSTERING_H

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>
#include "point_cloud_filter.h"

namespace mmwave_obstacle_avoidance_ros2 {

/**
 * @brief 聚类结果结构
 */
struct ClusterResult {
    pcl::PointCloud<PointXYZP>::Ptr cloud;
    Eigen::Vector3f centroid;     // 聚类中心
    Eigen::Vector3f min_bound;    // 最小边界
    Eigen::Vector3f max_bound;    // 最大边界
    float volume;                 // 体积
    int point_count;              // 点数
};

/**
 * @brief L2 空间聚类类
 * 功能：统计学离群点移除(SOR) + 欧几里得聚类/DBSCAN
 */
class SpatialClustering {
public:
    /**
     * @brief 构造函数
     */
    SpatialClustering();

    /**
     * @brief 设置SOR参数
     * @param mean_k 临近点数
     * @param stddev_mul 标准差倍数
     */
    void setSORParameters(int mean_k, double stddev_mul);

    /**
     * @brief 设置聚类参数
     * @param cluster_tolerance 聚类距离阈值(m)
     * @param min_cluster_size 最小聚类点数
     * @param max_cluster_size 最大聚类点数
     */
    void setClusterParameters(double cluster_tolerance, int min_cluster_size, int max_cluster_size);

    /**
     * @brief 设置自适应聚类参数
     * @param tolerance_base 基础容差(m)
     * @param tolerance_k 距离比例系数
     * @param max_tolerance 最大容差上限(m)，防止过大
     * @param min_cluster_size 最小聚类点数
     * @param max_cluster_size 最大聚类点数
     */
    void setClusterParameters(double tolerance_base, double tolerance_k, double max_tolerance, 
                              int min_cluster_size, int max_cluster_size);

    /**
     * @brief 执行空间聚类
     * @param input 输入点云
     * @param clusters 输出聚类结果
     * @return 聚类数量
     */
    int clusterPointCloud(const pcl::PointCloud<PointXYZP>::Ptr& input,
                          std::vector<ClusterResult>& clusters);

    /**
     * @brief 获取离群点移除后的点云
     */
    pcl::PointCloud<PointXYZP>::Ptr getFilteredCloud() const { return filtered_cloud_; }

    /**
     * @brief 获取统计信息
     */
    void getStatistics(int& input_points, int& outliers_removed, int& cluster_count) const;

private:
    // SOR参数
    int sor_mean_k_;
    double sor_stddev_mul_;

    // 聚类参数
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double cluster_tolerance_base_; // 基础容差
    double cluster_tolerance_k_;    // 随距离增加的比例系数
    double cluster_max_tolerance_;  // 容差上限

    // 统计信息
    int input_points_;
    int outliers_removed_;
    int cluster_count_;

    // 中间结果
    pcl::PointCloud<PointXYZP>::Ptr filtered_cloud_;

    pcl::search::KdTree<PointXYZP>::Ptr tree_;
    /**
     * @brief 统计学离群点移除
     */
    void removeOutliers(const pcl::PointCloud<PointXYZP>::Ptr& input,
                        pcl::PointCloud<PointXYZP>::Ptr& output);

    /**
     * @brief 欧几里得聚类
     */
    void euclideanClustering(const pcl::PointCloud<PointXYZP>::Ptr& input,
                             std::vector<pcl::PointIndices>& cluster_indices);

    /**
     * @brief 计算聚类属性
     */
    void computeClusterProperties(const pcl::PointCloud<PointXYZP>::Ptr& cloud,
                                  const pcl::PointIndices& indices,
                                  ClusterResult& result);
};

} // namespace mmwave_obstacle_avoidance_ros2

#endif // MMWAVE_OBSTACLE_AVOIDANCE_ROS2_SPATIAL_CLUSTERING_H
