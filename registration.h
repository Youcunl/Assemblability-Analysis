//3d registration algorithm

#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf("4-by-4 matrix :\n");
    printf("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3));
    printf("T = | %6.3f %6.3f %6.3f %6.3f| \n", matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3));
    printf("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
    printf("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3, 3));
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

double icpRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr core_pcd, pcl::PointCloud<pcl::PointXYZ>::Ptr male_pcd,
                       int maxIter, Eigen::Matrix4d &transformation_matrix)
{
    transformation_matrix = Eigen::Matrix4d::Identity();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(maxIter);
    icp.setInputSource(core_pcd); //core
    icp.setInputTarget(male_pcd); //male
    icp.align(*core_pcd);         //core
    //icp.setMaximumIterations(1);
    //icp.align(*core_pcd);

    std::cout << "converged? " << icp.hasConverged() << std::endl;
    std::cout << "icp score is " << icp.getFitnessScore() << std::endl;
    transformation_matrix = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transformation_matrix);

    return icp.getFitnessScore();
}

double icpRegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr core_pcd, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr male_pcd,
                       int maxIter, Eigen::Matrix4d &transformation_matrix)
{
    transformation_matrix = Eigen::Matrix4d::Identity();
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
    icp.setMaximumIterations(maxIter);
    icp.setInputSource(core_pcd); //core
    icp.setInputTarget(male_pcd); //male
    icp.align(*core_pcd);         //core
    //icp.setMaximumIterations(1);
    //icp.align(*core_pcd);

    std::cout << "converged? " << icp.hasConverged() << std::endl;
    std::cout << "icp score is " << icp.getFitnessScore() << std::endl;
    transformation_matrix = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transformation_matrix);

    return icp.getFitnessScore();
}

#endif