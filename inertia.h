#ifndef INERTIA_H
#define INERTIA_H

#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>

using namespace std;

pcl::PointXYZ scalar(pcl::PointXYZ start, pcl::PointXYZ end, double k){
	pcl::PointXYZ res((end.x - start.x)*k + end.x, (end.y - start.y)*k + end.y, (end.z - start.z)*k + end.z);
	return res;
}


vector<Eigen::Vector3d> get_inertia(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr){

  	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (cloud_ptr);
	feature_extractor.compute ();
 
	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
 
 
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
 
 
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;
 
	feature_extractor.getMomentOfInertia (moment_of_inertia);
	feature_extractor.getEccentricity (eccentricity);
	feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter (mass_center);

	Eigen::Vector3d a(double(mass_center(0)), double(mass_center(1)), double(mass_center(2)));
	Eigen::Vector3d b(double(major_vector(0)), double(major_vector(1)), double(major_vector(2)));
	Eigen::Vector3d c(double(middle_vector(0)), double(middle_vector(1)), double(middle_vector(2)));
	Eigen::Vector3d d(double(minor_vector(0)), double(minor_vector(1)), double(minor_vector(2)));

	Eigen::Vector3d min_BB(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
	Eigen::Vector3d max_BB(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);

	vector<Eigen::Vector3d> ret = {a,b,c,d,min_BB,max_BB};
	return ret;
}

#endif