#include "include/front_end.hpp"

FrontEnd::FrontEnd()
{
}

void FrontEnd::FilterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud,
                                float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);                   // 输入点云
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置体素网格大小
    voxel_filter.filter(*filtered_cloud);                      // 执行滤波
}

void FrontEnd::ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                         const Eigen::Matrix4d &initial_transform,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned_cloud,
                         Eigen::Matrix4d &final_transform)
{
    // Step 1: Apply the initial transform to the source cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *transformed_source, initial_transform.cast<float>());

    // Step 2: Set up the ICP object
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(transformed_source); // 设置初始变换后的源点云
    icp.setInputTarget(target_cloud);       // 设置目标点云

    // 设置 ICP 算法的参数
    icp.setMaxCorrespondenceDistance(1);
    icp.setMaximumIterations(500);         // 最大迭代次数为 50
    icp.setTransformationEpsilon(1e-8);   // 设置收敛条件，连续两次变换的差异阈值
    icp.setEuclideanFitnessEpsilon(1e-5); // 设置最终误差的收敛条件

    // Step 3: Perform ICP alignment
    icp.align(*aligned_cloud);

    // Check if ICP converged
    if (icp.hasConverged())
    {
        std::cout << "ICP converged." << std::endl;
        std::cout << "ICP fitness score: " << icp.getFitnessScore() << std::endl;

        // Step 4: Get the final transformation matrix (4x4)
        Eigen::Matrix4f final_transform_f = icp.getFinalTransformation();

        // Convert final transformation to Eigen::Matrix4d
        final_transform = final_transform_f.cast<double>();

        std::cout << "Final transformation matrix:\n"
                  << final_transform << std::endl;
    }
    else
    {
        std::cout << "ICP did not converge." << std::endl;
    }
}
