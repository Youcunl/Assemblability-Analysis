#ifndef PART_MATCH_SCREW_H
#define PART_MATCH_SCREW_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <igl/viewer/Viewer.h>
#include <igl/writeOBJ.h>
#include <string>
#include <iostream>
#include <cmath>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr draw_spiral(Eigen::Vector3d p, double r, double H, Eigen::MatrixXd &output){
    Eigen::MatrixXd spiral;
    double z = p(1);

    for(double ang = 0; ang < 10 * M_PI; ang += 0.05) {
        if(z+0.025 > p(1)+H)
            break;
        Eigen::Vector3d tmp(r *sin(ang)+p(0), z, r *cos(ang)+p(2));
        z += 0.025;
        spiral.conservativeResize(spiral.rows()+1, 3);
        spiral.bottomRows(1) = tmp.transpose();
    }
    output = spiral;

    pcl::PointCloud<pcl::PointXYZ> cloud2;
    cloud2.width    = spiral.rows();
    cloud2.height   = 1;
    cloud2.is_dense = false;
    cloud2.points.resize (cloud2.width * cloud2.height);
    for (size_t j = 0; j < cloud2.points.size (); ++j)
    {
        cloud2.points[j].x = spiral(j,0);
        cloud2.points[j].y = spiral(j,1);
        cloud2.points[j].z = spiral(j,2);
    }

    return cloud2.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr draw_circle(Eigen::Vector3d p, double r, Eigen::MatrixXd &output){
    Eigen::MatrixXd spiral;
    for(double ang = 0; ang < 2 * M_PI; ang += 0.2) {
        Eigen::Vector3d tmp(r *sin(ang)+p(0), p(1), r *cos(ang)+p(2));
        spiral.conservativeResize(spiral.rows()+1, 3);
        spiral.bottomRows(1) = tmp.transpose();
    }
    output = spiral;

    pcl::PointCloud<pcl::PointXYZ> cloud2;
    cloud2.width    = spiral.rows();
    cloud2.height   = 1;
    cloud2.is_dense = false;
    cloud2.points.resize (cloud2.width * cloud2.height);
    for (size_t j = 0; j < cloud2.points.size (); ++j)
    {
        cloud2.points[j].x = spiral(j,0);
        cloud2.points[j].y = spiral(j,1);
        cloud2.points[j].z = spiral(j,2);
    }

    return cloud2.makeShared();
}

void cloud2mesh(vector<Eigen::MatrixXd> spirals, Eigen::MatrixXd &output_v, Eigen::MatrixXi &output_f){
    Eigen::MatrixXd v;
    Eigen::MatrixXi f;
    for(int i = 0;i<spirals.size();++i){
        v.conservativeResize(v.rows() + spirals[i].rows(), 3);
        v.bottomRows(spirals[i].rows()) = spirals[i];
    }

    vector<int> starters;
    vector<int> ends;
    int starter = 0;
    for(int i = 0;i<spirals.size();++i){
        Eigen::MatrixXd front = spirals[i];
        starters.push_back(starter);
        if(i!=spirals.size()-1){
            for(int j = starter;j<starter+front.rows();++j){
                Eigen::MatrixXi tmp(2, 3);
                if(j!=starter + front.rows()-1){
                    tmp << j,  j+front.rows(), j+front.rows()+1, j, j+front.rows()+1, j+1;
                    f.conservativeResize(f.rows()+2, 3);
                    f.bottomRows(2) = tmp;
                }
            }
        }else{
            for(int j = starter;j<starter+front.rows();++j){
                Eigen::MatrixXi tmp(2, 3);
                if(j!=starter + front.rows()-1){
                    tmp << j, j-starter, j-starter+1, j, j-starter+1, j+1;
                    f.conservativeResize(f.rows()+2, 3);
                    f.bottomRows(2) = tmp;
                }
            }
        }

        starter += front.rows();
        ends.push_back(starter-1);
    }

    cout<<starters.size()<<endl;
    cout<<ends.size()<<endl;
    for(int i=0;i<starters.size()-2;++i){
        Eigen::MatrixXi tmp(2, 3);
        tmp<<starters[0], starters[i+2], starters[i+1], ends[0], ends[i+1], ends[i+2];
        f.conservativeResize(f.rows()+2, 3);
        f.bottomRows(2) = tmp;
    }


    output_v = v;
    output_f = f;
}


void draw_screw(Eigen::Vector3d center, double radius, double h, bool show, Eigen::MatrixXd &ret_v, Eigen::MatrixXi &ret_f){

    double r = 0.25;
    Eigen::MatrixXd output;

    pcl::PointCloud<pcl::PointXYZ>::Ptr base_circle = draw_circle(center, r, output);
    //cout<<output.rows()<<endl;

    vector<Eigen::MatrixXd> spirals;
    for(int i =0;i<output.rows();++i){
        Eigen::MatrixXd output2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr spiral = draw_spiral(output.row(i), radius, h, output2);
        spirals.push_back(output2);
        //cout<<spiral->points.size()<<endl;
    }

    cout<<"ok"<<endl;
//    Eigen::MatrixXd v;
//    Eigen::MatrixXi f;
    cloud2mesh(spirals, ret_v, ret_f);
    //igl::writeOBJ("./tmp.obj", v, f);

    if(show){
        igl::viewer::Viewer viewer2;
        viewer2.data.set_mesh(ret_v, ret_f);
        viewer2.data.set_face_based(true);
        viewer2.core.orthographic = true;
        viewer2.core.show_lines = false;
        viewer2.launch();
    }
}

void get_center(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double &radius, Eigen::Vector3d &center){

    double x1 = p1(0);
    double x2 = p2(0);
    double x3 = p3(0);

    double y1 = p1(2);
    double y2 = p2(2);
    double y3 = p3(2);

    double a=sqrt( (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2) );
    double b=sqrt( (x1-x3)*(x1-x3)+(y1-y3)*(y1-y3) );
    double c=sqrt( (x2-x3)*(x2-x3)+(y2-y3)*(y2-y3) );
    double p=(a+b+c)/2;
    double S=sqrt( p*(p-a)*(p-b)*(p-c) );
    radius=a*b*c/(4*S);

    double t1=x1*x1+y1*y1;
    double t2=x2*x2+y2*y2;
    double t3=x3*x3+y3*y3;
    double temp=x1*y2+x2*y3+x3*y1-x1*y3-x2*y1-x3*y2;
    double x=(t2*y3+t1*y2+t3*y1-t2*y1-t3*y2-t1*y3)/temp/2;
    double y=(t3*x2+t2*x1+t1*x3-t1*x2-t2*x3-t3*x1)/temp/2;

    Eigen::Vector3d ccc(x, p1(1) ,y);
    center = ccc;
}
#endif //PART_MATCH_SCREW_H
