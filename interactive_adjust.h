#ifndef PART_MATCH_INTERACTIVE_ADJUST_H
#define PART_MATCH_INTERACTIVE_ADJUST_H

#include <igl/viewer/Viewer.h>
#include <igl/copyleft/cgal/intersect_other.h>
#include <igl/writeOBJ.h>

#include "inertia.h"
#include "alignment.h"
//#include "mesh_sampling.h"

using namespace std;

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier);

vector<Eigen::Matrix4d> adjusted_T;

Eigen::MatrixXd female_v;
Eigen::MatrixXi female_f;
vector<Eigen::MatrixXd> males_v;
vector<Eigen::MatrixXi> males_f;
vector<Eigen::Matrix4d> original_T;
vector<Eigen::Vector3d> axiss;
vector<Eigen::Vector3d> centers;
vector<Eigen::RowVector3d> colors;
vector<int> assign;

void GUI(Eigen::MatrixXd f_v, Eigen::MatrixXi f_f, vector<Eigen::MatrixXd> ms_v, vector<Eigen::MatrixXi> ms_f,
         vector<int> assignment, vector< vector<Eigen::Matrix4d> > T, vector<Eigen::Vector3d> as,
         vector<Eigen::MatrixXd> &ret_v, vector<Eigen::MatrixXi> &ret_f){
    female_v = f_v;
    female_f = f_f;
    males_v = ms_v;
    males_f = ms_f;
    original_T = vector<Eigen::Matrix4d>(4, Eigen::Matrix4d::Identity());
    for(int i=0;i<assignment.size();++i){
        original_T[assignment[i]] = T[i][assignment[i]];
    }
    axiss = as;
    assign = assignment;
    colors = {Eigen::RowVector3d(0.1, 0.9, 0.1), Eigen::RowVector3d(0.1, 0.1, 0.9),
              Eigen::RowVector3d(0.9, 0.1, 0.9),Eigen::RowVector3d(0.1, 0.9, 0.9)};

    for(int i = 0;i<males_v.size();++i){
        Eigen::MatrixXd male_v = males_v[i];
        //transpose the transformation matrix
        Eigen::Matrix4d transformation_matrix_T = (original_T[i]).transpose();
        //build homogenous vertex list x-by-4
        Eigen::MatrixXd male_homo_v(male_v.rows(), 4);
        male_homo_v.leftCols(3) = male_v;
        male_homo_v.rightCols(1) = Eigen::MatrixXd::Constant(male_homo_v.rows(), 1, 1);
        male_homo_v = male_homo_v * transformation_matrix_T;
        males_v[i] = male_homo_v.leftCols(3);
        centers.push_back(get_inertia(vertex2cloud(males_v[i]))[0]);
    }

    //initialize
    Eigen::MatrixXd show_mesh_v;
    Eigen::MatrixXi show_mesh_f;
    Eigen::MatrixXd show_mesh_color;
    //assign
    int cache= 0;
    for(int i = 0; i<males_v.size(); ++i){
        Eigen::MatrixXd tmp_male_v = males_v[i];
        Eigen::MatrixXi male_f = males_f[i];
        //merge the meshes
        show_mesh_v.conservativeResize(show_mesh_v.rows()+tmp_male_v.rows(), 3);
        show_mesh_v.bottomRows(tmp_male_v.rows())=tmp_male_v;
        show_mesh_f.conservativeResize(show_mesh_f.rows()+male_f.rows(), 3);
        show_mesh_f.bottomRows(male_f.rows())=male_f.array()+cache;
        show_mesh_color.conservativeResize(show_mesh_color.rows()+male_f.rows(), 3);
        show_mesh_color.bottomRows(male_f.rows())=colors[i].replicate(male_f.rows(), 1);

        cache += tmp_male_v.rows();
    }

    //merge female
    show_mesh_v.conservativeResize(show_mesh_v.rows()+female_v.rows(), 3);
    show_mesh_v.bottomRows(female_v.rows())=female_v;
    show_mesh_f.conservativeResize(show_mesh_f.rows()+female_f.rows(), 3);
    show_mesh_f.bottomRows(female_f.rows())=female_f.array()+cache;
    show_mesh_color.conservativeResize(show_mesh_color.rows()+female_f.rows(), 3);
    show_mesh_color.bottomRows(female_f.rows())=
            Eigen::RowVector3d(0.9,0.9,0.1).replicate(female_f.rows(), 1);


    igl::viewer::Viewer viewer;
    viewer.callback_key_down = &key_down;
    viewer.data.set_mesh(show_mesh_v, show_mesh_f);
    viewer.data.set_colors(show_mesh_color);
    viewer.data.set_face_based(true);
    viewer.core.orthographic = true; //set to ortho view
    viewer.core.show_lines = false;
    viewer.launch();

    cout<<"Return and saving..."<<endl;
    ret_v = males_v;
    ret_f = males_f;
}


bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier)
{
    std::cout<<"Key: "<<key<<" "<<(unsigned int)key<<std::endl;
    if (key == '1' || key == '2')
    {
        Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
        if(key == '1')
            translation.topRightCorner(3, 1) = axiss[0];
        else
            translation.topRightCorner(3, 1) = -1*axiss[0];

        Eigen::MatrixXd this_tmp_homo_v(males_v[0].rows(), 4);
        this_tmp_homo_v.leftCols(3) = males_v[0];
        this_tmp_homo_v.rightCols(1) = Eigen::MatrixXd::Constant(this_tmp_homo_v.rows(), 1, 1);
        this_tmp_homo_v = this_tmp_homo_v * (translation).transpose();
        males_v[0] = this_tmp_homo_v.leftCols(3);

        Eigen::MatrixXi IF;
        if(igl::copyleft::cgal::intersect_other(males_v[0],males_f[0],female_v,female_f,true,IF)){
            colors[0] = Eigen::RowVector3d(0.9, 0.1, 0.1);
        }else{
            colors[0] = Eigen::RowVector3d(0.1, 0.9, 0.1);
        }

        Eigen::MatrixXd show_mesh_v;
        Eigen::MatrixXi show_mesh_f;
        Eigen::MatrixXd show_mesh_color;

        int cache= 0;
        for(int i = 0; i<males_v.size(); ++i){
            Eigen::MatrixXd tmp_male_v = males_v[i];
            Eigen::MatrixXi male_f = males_f[i];
            //merge the meshes
            show_mesh_v.conservativeResize(show_mesh_v.rows()+tmp_male_v.rows(), 3);
            show_mesh_v.bottomRows(tmp_male_v.rows())=tmp_male_v;
            show_mesh_f.conservativeResize(show_mesh_f.rows()+male_f.rows(), 3);
            show_mesh_f.bottomRows(male_f.rows())=male_f.array()+cache;
            show_mesh_color.conservativeResize(show_mesh_color.rows()+male_f.rows(), 3);
            show_mesh_color.bottomRows(male_f.rows())=colors[i].replicate(male_f.rows(), 1);

            cache += tmp_male_v.rows();
        }

        //merge female
        show_mesh_v.conservativeResize(show_mesh_v.rows()+female_v.rows(), 3);
        show_mesh_v.bottomRows(female_v.rows())=female_v;
        show_mesh_f.conservativeResize(show_mesh_f.rows()+female_f.rows(), 3);
        show_mesh_f.bottomRows(female_f.rows())=female_f.array()+cache;
        show_mesh_color.conservativeResize(show_mesh_color.rows()+female_f.rows(), 3);
        show_mesh_color.bottomRows(female_f.rows())=
                Eigen::RowVector3d(0.9,0.9,0.1).replicate(female_f.rows(), 1);

        viewer.data.clear();
        viewer.data.set_mesh(show_mesh_v, show_mesh_f);
        viewer.data.set_colors(show_mesh_color);
        viewer.data.set_face_based(true);
        viewer.core.orthographic = true;
        viewer.core.show_lines = false;
    }
    else if (key == '3' || key == '4')
    {
        Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
        if(key == '3')
            translation.topRightCorner(3, 1) = axiss[1];
        else
            translation.topRightCorner(3, 1) = -1*axiss[1];

        Eigen::MatrixXd this_tmp_homo_v(males_v[1].rows(), 4);
        this_tmp_homo_v.leftCols(3) = males_v[1];
        this_tmp_homo_v.rightCols(1) = Eigen::MatrixXd::Constant(this_tmp_homo_v.rows(), 1, 1);
        this_tmp_homo_v = this_tmp_homo_v * (translation).transpose();
        males_v[1] = this_tmp_homo_v.leftCols(3);

        Eigen::MatrixXi IF;
        if(igl::copyleft::cgal::intersect_other(males_v[1],males_f[1],female_v,female_f,true,IF)){
            colors[1] = Eigen::RowVector3d(0.9, 0.1, 0.1);
        }else{
            colors[1] = Eigen::RowVector3d(0.1, 0.1, 0.9);
        }

        Eigen::MatrixXd show_mesh_v;
        Eigen::MatrixXi show_mesh_f;
        Eigen::MatrixXd show_mesh_color;

        int cache= 0;
        for(int i = 0; i<males_v.size(); ++i){
            Eigen::MatrixXd tmp_male_v = males_v[i];
            Eigen::MatrixXi male_f = males_f[i];
            //merge the meshes
            show_mesh_v.conservativeResize(show_mesh_v.rows()+tmp_male_v.rows(), 3);
            show_mesh_v.bottomRows(tmp_male_v.rows())=tmp_male_v;
            show_mesh_f.conservativeResize(show_mesh_f.rows()+male_f.rows(), 3);
            show_mesh_f.bottomRows(male_f.rows())=male_f.array()+cache;
            show_mesh_color.conservativeResize(show_mesh_color.rows()+male_f.rows(), 3);
            show_mesh_color.bottomRows(male_f.rows())=colors[i].replicate(male_f.rows(), 1);

            cache += tmp_male_v.rows();
        }

        //merge female
        show_mesh_v.conservativeResize(show_mesh_v.rows()+female_v.rows(), 3);
        show_mesh_v.bottomRows(female_v.rows())=female_v;
        show_mesh_f.conservativeResize(show_mesh_f.rows()+female_f.rows(), 3);
        show_mesh_f.bottomRows(female_f.rows())=female_f.array()+cache;
        show_mesh_color.conservativeResize(show_mesh_color.rows()+female_f.rows(), 3);
        show_mesh_color.bottomRows(female_f.rows())=
                Eigen::RowVector3d(0.9,0.9,0.1).replicate(female_f.rows(), 1);

        viewer.data.clear();
        viewer.data.set_mesh(show_mesh_v, show_mesh_f);
        viewer.data.set_colors(show_mesh_color);
        viewer.data.set_face_based(true);
        viewer.core.orthographic = true;
        viewer.core.show_lines = false;
    }
    else if (key == '5' || key == '6')
    {
        Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
        if(key == '5')
            translation.topRightCorner(3, 1) = axiss[2];
        else
            translation.topRightCorner(3, 1) = -1*axiss[2];

        Eigen::MatrixXd this_tmp_homo_v(males_v[2].rows(), 4);
        this_tmp_homo_v.leftCols(3) = males_v[2];
        this_tmp_homo_v.rightCols(1) = Eigen::MatrixXd::Constant(this_tmp_homo_v.rows(), 1, 1);
        this_tmp_homo_v = this_tmp_homo_v * (translation).transpose();
        males_v[2] = this_tmp_homo_v.leftCols(3);

        Eigen::MatrixXi IF;
        if(igl::copyleft::cgal::intersect_other(males_v[2],males_f[2],female_v,female_f,true,IF)){
            colors[2] = Eigen::RowVector3d(0.9, 0.1, 0.1);
        }else{
            colors[2] = Eigen::RowVector3d(0.9, 0.1, 0.9);
        }

        Eigen::MatrixXd show_mesh_v;
        Eigen::MatrixXi show_mesh_f;
        Eigen::MatrixXd show_mesh_color;

        int cache= 0;
        for(int i = 0; i<males_v.size(); ++i){
            Eigen::MatrixXd tmp_male_v = males_v[i];
            Eigen::MatrixXi male_f = males_f[i];
            //merge the meshes
            show_mesh_v.conservativeResize(show_mesh_v.rows()+tmp_male_v.rows(), 3);
            show_mesh_v.bottomRows(tmp_male_v.rows())=tmp_male_v;
            show_mesh_f.conservativeResize(show_mesh_f.rows()+male_f.rows(), 3);
            show_mesh_f.bottomRows(male_f.rows())=male_f.array()+cache;
            show_mesh_color.conservativeResize(show_mesh_color.rows()+male_f.rows(), 3);
            show_mesh_color.bottomRows(male_f.rows())=colors[i].replicate(male_f.rows(), 1);

            cache += tmp_male_v.rows();
        }

        //merge female
        show_mesh_v.conservativeResize(show_mesh_v.rows()+female_v.rows(), 3);
        show_mesh_v.bottomRows(female_v.rows())=female_v;
        show_mesh_f.conservativeResize(show_mesh_f.rows()+female_f.rows(), 3);
        show_mesh_f.bottomRows(female_f.rows())=female_f.array()+cache;
        show_mesh_color.conservativeResize(show_mesh_color.rows()+female_f.rows(), 3);
        show_mesh_color.bottomRows(female_f.rows())=
                Eigen::RowVector3d(0.9,0.9,0.1).replicate(female_f.rows(), 1);

        viewer.data.clear();
        viewer.data.set_mesh(show_mesh_v, show_mesh_f);
        viewer.data.set_colors(show_mesh_color);
        viewer.data.set_face_based(true);
        viewer.core.orthographic = true;
        viewer.core.show_lines = false;
    }
    else if (key == '7' || key == '8')
    {
        Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
        if(key == '7')
            translation.topRightCorner(3, 1) = axiss[3];
        else
            translation.topRightCorner(3, 1) = -1*axiss[3];

        Eigen::MatrixXd this_tmp_homo_v(males_v[3].rows(), 4);
        this_tmp_homo_v.leftCols(3) = males_v[3];
        this_tmp_homo_v.rightCols(1) = Eigen::MatrixXd::Constant(this_tmp_homo_v.rows(), 1, 1);
        this_tmp_homo_v = this_tmp_homo_v * (translation).transpose();
        males_v[3] = this_tmp_homo_v.leftCols(3);

        Eigen::MatrixXi IF;
        if(igl::copyleft::cgal::intersect_other(males_v[3],males_f[3],female_v,female_f,true,IF)){
            colors[3] = Eigen::RowVector3d(0.9, 0.1, 0.1);
        }else{
            colors[3] = Eigen::RowVector3d(0.1, 0.9, 0.9);
        }

        Eigen::MatrixXd show_mesh_v;
        Eigen::MatrixXi show_mesh_f;
        Eigen::MatrixXd show_mesh_color;

        int cache= 0;
        for(int i = 0; i<males_v.size(); ++i){
            Eigen::MatrixXd tmp_male_v = males_v[i];
            Eigen::MatrixXi male_f = males_f[i];
            //merge the meshes
            show_mesh_v.conservativeResize(show_mesh_v.rows()+tmp_male_v.rows(), 3);
            show_mesh_v.bottomRows(tmp_male_v.rows())=tmp_male_v;
            show_mesh_f.conservativeResize(show_mesh_f.rows()+male_f.rows(), 3);
            show_mesh_f.bottomRows(male_f.rows())=male_f.array()+cache;
            show_mesh_color.conservativeResize(show_mesh_color.rows()+male_f.rows(), 3);
            show_mesh_color.bottomRows(male_f.rows())=colors[i].replicate(male_f.rows(), 1);

            cache += tmp_male_v.rows();
        }

        //merge female
        show_mesh_v.conservativeResize(show_mesh_v.rows()+female_v.rows(), 3);
        show_mesh_v.bottomRows(female_v.rows())=female_v;
        show_mesh_f.conservativeResize(show_mesh_f.rows()+female_f.rows(), 3);
        show_mesh_f.bottomRows(female_f.rows())=female_f.array()+cache;
        show_mesh_color.conservativeResize(show_mesh_color.rows()+female_f.rows(), 3);
        show_mesh_color.bottomRows(female_f.rows())=
                Eigen::RowVector3d(0.9,0.9,0.1).replicate(female_f.rows(), 1);

        viewer.data.clear();
        viewer.data.set_mesh(show_mesh_v, show_mesh_f);
        viewer.data.set_colors(show_mesh_color);
        viewer.data.set_face_based(true);
        viewer.core.orthographic = true;
        viewer.core.show_lines = false;
    }

    return false;
}

#endif //PART_MATCH_INTERACTIVE_ADJUST_H
