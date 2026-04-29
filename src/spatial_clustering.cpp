#define PCL_NO_PRECOMPILE

#include "mmwave_obstacle_avoidance_ros2/spatial_clustering.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/impl/pcl_base.hpp>
namespace mmwave_obstacle_avoidance_ros2 {

SpatialClustering::SpatialClustering()
    : sor_mean_k_(20),
      sor_stddev_mul_(1.0),
      cluster_tolerance_(0.3),
      cluster_tolerance_base_(0.1),    
      cluster_tolerance_k_(0.05),      
      cluster_max_tolerance_(1.5),     
      min_cluster_size_(5),
      max_cluster_size_(500),
      input_points_(0),
      outliers_removed_(0),
      cluster_count_(0) {
    filtered_cloud_.reset(new pcl::PointCloud<PointXYZP>);
}

void SpatialClustering::setSORParameters(int mean_k, double stddev_mul) {
    sor_mean_k_ = mean_k;
    sor_stddev_mul_ = stddev_mul;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "SOR parameters: mean_k=%d, stddev_mul=%.2f", mean_k, stddev_mul);
}

void SpatialClustering::setClusterParameters(double tolerance_base, double tolerance_k, double max_tolerance,
                                              int min_cluster_size, int max_cluster_size) {
    cluster_tolerance_base_ = tolerance_base;
    cluster_tolerance_k_ = tolerance_k;
    cluster_max_tolerance_ = max_tolerance;
    min_cluster_size_ = min_cluster_size;
    max_cluster_size_ = max_cluster_size;
    
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), 
        "Adaptive Cluster params: Base=%.2f, K=%.3f, MaxTol=%.2f, MinSize=%d",
        tolerance_base, tolerance_k, max_tolerance, min_cluster_size);
}

// 主聚类函数，返回聚类数量
// SOR ： 多径干扰（反射产生的虚假点）、传感器噪声（孤立点）、雨雾天气（杂波）
// 欧几里得聚类： 基于点之间的欧几里得距离进行聚类，适合分离不同物体的点云
// 计算聚类属性： 质心、边界、体积等，辅助后续决策
int SpatialClustering::clusterPointCloud(const pcl::PointCloud<PointXYZP>::Ptr& input,
                                         std::vector<ClusterResult>& clusters) {
    if (!input || input->points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"), "Input point cloud is empty for clustering");
        clusters.clear();
        return 0;
    }

    input_points_ = input->points.size();
    clusters.clear();

    // 第一步：统计学离群点移除
    // 实际飞行中，因为多径等会产生大量离群点，SOR可以有效提升聚类质量
    removeOutliers(input, filtered_cloud_);
    outliers_removed_ = input_points_ - filtered_cloud_->points.size();

    if (filtered_cloud_->points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"), "All points removed as outliers");
        return 0;
    }

    // 第二步：欧几里得聚类
    // 使用PCL的EuclideanClusterExtraction，基于KD树加速邻域搜索
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClustering(filtered_cloud_, cluster_indices);
    cluster_count_ = cluster_indices.size();

    // 第三步：提取聚类结果并计算属性
    // 计算每个聚类的点云、质心、边界和体积等属性，存储在ClusterResult结构中
    clusters.reserve(cluster_indices.size());
    for (const auto& indices : cluster_indices) {
        ClusterResult result;
        result.cloud.reset(new pcl::PointCloud<PointXYZP>);// 聚类点云集合
        computeClusterProperties(filtered_cloud_, indices, result);
        clusters.push_back(result);
    }

/*
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Clustering results: input=%d, outliers=%d, clusters=%d",
              input_points_, outliers_removed_, cluster_count_);
*/
    return clusters.size();
}

void SpatialClustering::removeOutliers(const pcl::PointCloud<PointXYZP>::Ptr& input,
                                        pcl::PointCloud<PointXYZP>::Ptr& output) {
    if (sor_mean_k_ <= 0) {
        // 跳过SOR过滤
        *output = *input;
        return;
    }

    pcl::StatisticalOutlierRemoval<PointXYZP> sor;
    sor.setInputCloud(input);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_stddev_mul_);
    sor.filter(*output);
}

void SpatialClustering::euclideanClustering(const pcl::PointCloud<PointXYZP>::Ptr& input,
                                             std::vector<pcl::PointIndices>& cluster_indices) {
    cluster_indices.clear();
    if (input->points.empty()) return;

    typename pcl::search::KdTree<PointXYZP>::Ptr tree(new pcl::search::KdTree<PointXYZP>);
    tree->setInputCloud(input);

    const size_t n = input->points.size();
    std::vector<bool> processed(n, false);
    std::vector<int> nn_indices;
    std::vector<float> nn_sqr_dists;
    nn_indices.reserve(64);
    nn_sqr_dists.reserve(64);

    std::vector<int> seed_queue;
    seed_queue.reserve(n);  

    for (size_t i = 0; i < n; ++i) {
        if (processed[i]) continue;

        seed_queue.clear();
        seed_queue.push_back(static_cast<int>(i));
        processed[i] = true;

        size_t sq_idx = 0;
        while (sq_idx < seed_queue.size()) {
            int p_idx = seed_queue[sq_idx];
            const auto& p = input->points[p_idx];

            float dist = std::sqrt(p.x * p.x + p.y * p.y);
            float adaptive_tolerance = cluster_tolerance_base_ + cluster_tolerance_k_ * dist;
            if (adaptive_tolerance > cluster_max_tolerance_)
                adaptive_tolerance = cluster_max_tolerance_;

            if (tree->radiusSearch(p, adaptive_tolerance, nn_indices, nn_sqr_dists) > 0) {
                for (int nb : nn_indices) {
                    if (!processed[nb]) {
                        seed_queue.push_back(nb);
                        processed[nb] = true;
                    }
                }
            }
            ++sq_idx;
        }

        if ((int)seed_queue.size() >= min_cluster_size_ &&
            (int)seed_queue.size() <= max_cluster_size_) {
            pcl::PointIndices r;
            r.indices = seed_queue;
            cluster_indices.push_back(std::move(r));  
        }
    }
}



void SpatialClustering::computeClusterProperties(const pcl::PointCloud<PointXYZP>::Ptr& cloud,
                                                  const pcl::PointIndices& indices,
                                                  ClusterResult& result) {
    // 提取聚类点云
    for (int idx : indices.indices) {
        result.cloud->points.push_back(cloud->points[idx]);
    }
    result.cloud->width = result.cloud->points.size();
    result.cloud->height = 1;
    result.cloud->is_dense = true;
    result.point_count = result.cloud->points.size();

    // 计算聚类中心和边界
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_pt(std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max());
    Eigen::Vector3f max_pt(std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest());

    for (const auto& point : result.cloud->points) {
        sum += Eigen::Vector3f(point.x, point.y, point.z);
        
        min_pt.x() = std::min(min_pt.x(), point.x);
        min_pt.y() = std::min(min_pt.y(), point.y);
        min_pt.z() = std::min(min_pt.z(), point.z);
        
        max_pt.x() = std::max(max_pt.x(), point.x);
        max_pt.y() = std::max(max_pt.y(), point.y);
        max_pt.z() = std::max(max_pt.z(), point.z);
    }

    result.centroid = sum / result.point_count;// 聚类中所有点的几何平均中心点
    result.min_bound = min_pt;// 聚类中所有点的最小边界点
    result.max_bound = max_pt;// 聚类中所有点的最大边界点

    // 计算体积(简化为包围盒体积)
    Eigen::Vector3f size = max_pt - min_pt;
    result.volume = size.x() * size.y() * size.z();
}

void SpatialClustering::getStatistics(int& input_points,
                                       int& outliers_removed,
                                       int& cluster_count) const {
    input_points = input_points_;
    outliers_removed = outliers_removed_;
    cluster_count = cluster_count_;
}

} // namespace mmwave_obstacle_avoidance_ros2
