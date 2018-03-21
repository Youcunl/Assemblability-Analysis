#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

class Transformation{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
	Eigen::Matrix4d matrix;

public:
	Transformation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
		this->cloud = cloud;
	}

	Eigen::Matrix4d get_matrix(){return matrix;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud(){return transformed_cloud;}

	void do_transformation(double x, double y, double z, char axis, double degree){
		matrix = translation(x, y, z)*rotation(axis, degree);
		pcl::transformPointCloud(*cloud, *transformed_cloud, matrix);
	}
    void do_transformation(double x, double y, double z){
        matrix = translation(x, y, z);
        pcl::transformPointCloud(*cloud, *transformed_cloud, matrix);
    }
    void do_transformation(char axis, double degree){
        matrix = rotation(axis, degree);
        pcl::transformPointCloud(*cloud, *transformed_cloud, matrix);
    }

private:
	Eigen::Matrix4d translation(double x, double y, double z){
		Eigen::Matrix4d tmp_translation;
    	tmp_translation << 1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1;
    	return tmp_translation;
	}

	Eigen::Matrix4d rotation(char axis, double angle){
		double theta = M_PI*angle/180;
		Eigen::Matrix4d tmp_rotation;

	    if(axis == 'x')
	    	tmp_rotation << 1,0,0,0,0,cos(theta),-sin(theta),0,0,sin(theta),cos(theta),0,0,0,0,1;
	    else if(axis == 'y')
	    	tmp_rotation << cos(theta),0,sin(theta),0,0,1,0,0,-sin(theta),0,cos(theta),0,0,0,0,1;
	    else
	    	tmp_rotation << cos(theta),-sin(theta),0,0,sin(theta),cos(theta),0,0,0,0,1,0,0,0,0,1;

	    return tmp_rotation; 
	}
};
#endif