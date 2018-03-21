#ifndef PART_MATCH_ALIGNMENT_H
#define PART_MATCH_ALIGNMENT_H

#include <iostream>
#include <cmath>
#include <igl/axis_angle_to_quat.h>
#include <pcl/common/transforms.h>
#include "inertia.h"

using namespace std;

class Align{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    vector<Eigen::Matrix4d> matrices;

private:
    Eigen::Vector3d center1;
    Eigen::Vector3d center2;
    Eigen::Vector3d axiss1;
    vector<Eigen::Vector3d> axiss2;

public:
    Align(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target){
        this->source = source;
        this->target = target;
        vector<Eigen::Vector3d> tmp1, tmp2;
        tmp1 = get_inertia(source);
        tmp2 = get_inertia(target);
        center1 = tmp1[0];
        center2 = tmp2[0];
        axiss1 = tmp1[1];
        axiss2.push_back(tmp2[1]);
        axiss2.push_back(tmp2[2]);
        axiss2.push_back(tmp2[3]);
    }

    vector<Eigen::Matrix4d> get_matrix(){return matrices;}
    Eigen::Vector3d get_axis(int index){
        Eigen::Vector3d tmp = axiss2[index];
        tmp.normalize();
        return tmp;
    }
    Eigen::Vector3d get_center(){return center1;}

public:
    void run(){
        for(int j = 0;j<axiss2.size();++j){
            Eigen::Matrix4d tmp_T;
            tmp_T = compute(axiss1, axiss2[j]);
            matrices.push_back(tmp_T);
        }
    }

    Eigen::Matrix4d compute(Eigen::Vector3d axis1, Eigen::Vector3d axis2){
        Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d translation1 = Eigen::Matrix4d::Identity();;
        Eigen::Matrix4d translation2 = Eigen::Matrix4d::Identity();;
        Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        Eigen::Vector3d axis = axis1.cross(axis2);
        axis.normalize();
        double angle = acos(axis1.dot(axis2)/((axis1.dot(axis1))*(axis2.dot(axis2))));
        Eigen::AngleAxisd m = Eigen::AngleAxisd(angle+M_PI,axis);

        translation1.topRightCorner(3, 1) = -1*center1;
        rotation.topLeftCorner(3, 3) = m.toRotationMatrix();
        translation2.topRightCorner(3, 1) = center2;

        T = translation2*rotation*translation1;

        return T;
    }

};

#endif //PART_MATCH_ALIGNMENT_H
