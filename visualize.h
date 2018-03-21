#ifndef PART_MATCH_VISUALIZE_H
#define PART_MATCH_VISUALIZE_H

#include <igl/viewer/Viewer.h>

using namespace std;

void visualize(vector<int> assignment, vector< vector<Eigen::Matrix4d> > T,
               Eigen::MatrixXd female_v, Eigen::MatrixXi female_f,
               vector<Eigen::MatrixXd> males_v, vector<Eigen::MatrixXi> males_f){
    //initialize
    Eigen::MatrixXd show_mesh_v;
    Eigen::MatrixXi show_mesh_f;
    Eigen::MatrixXd show_mesh_color;

    //assign
    int cache= 0;
    //int cache_f = 0;
    for(int i = 0; i<males_v.size(); ++i){
        //query male
        Eigen::MatrixXd male_v = males_v[assignment[i]];
        Eigen::MatrixXi male_f = males_f[assignment[i]];
        //transpose the transformation matrix
        Eigen::Matrix4d transformation_matrix_T = (T[i][assignment[i]]).transpose();
        //build homogenous vertex list x-by-4
        Eigen::MatrixXd male_homo_v(male_v.rows(), 4);
        male_homo_v.leftCols(3) = male_v;
        male_homo_v.rightCols(1) = Eigen::MatrixXd::Constant(male_homo_v.rows(), 1, 1);
        male_homo_v = male_homo_v * transformation_matrix_T;
        //male part after transformation
        Eigen::MatrixXd tmp_male_v(male_v.rows(), 3);
        tmp_male_v.leftCols(3) = male_homo_v.leftCols(3);

        //merge the meshes
        show_mesh_v.conservativeResize(show_mesh_v.rows()+tmp_male_v.rows(), 3);
        show_mesh_v.bottomRows(tmp_male_v.rows())=tmp_male_v;
        show_mesh_f.conservativeResize(show_mesh_f.rows()+male_f.rows(), 3);
        show_mesh_f.bottomRows(male_f.rows())=male_f.array()+cache;
        show_mesh_color.conservativeResize(show_mesh_color.rows()+male_f.rows(), 3);
        show_mesh_color.bottomRows(male_f.rows())=
                Eigen::RowVector3d(((float)rand())/(float)RAND_MAX, ((float)rand())/(float)RAND_MAX, ((float)rand())/(float)RAND_MAX).replicate(male_f.rows(), 1);

        cache += tmp_male_v.rows();
        //cache_f += male_f.rows();
    }

    //merge female
    show_mesh_v.conservativeResize(show_mesh_v.rows()+female_v.rows(), 3);
    show_mesh_v.bottomRows(female_v.rows())=female_v;
    show_mesh_f.conservativeResize(show_mesh_f.rows()+female_f.rows(), 3);
    show_mesh_f.bottomRows(female_f.rows())=female_f.array()+cache;
    show_mesh_color.conservativeResize(show_mesh_color.rows()+female_f.rows(), 3);
    show_mesh_color.bottomRows(female_f.rows())=
            Eigen::RowVector3d(((float) rand())/(float) RAND_MAX, ((float)rand()) / (float)RAND_MAX, ((float)rand()) / (float)RAND_MAX).replicate(female_f.rows(), 1);


    igl::viewer::Viewer viewer;
    viewer.data.set_mesh(show_mesh_v, show_mesh_f);
    viewer.data.set_colors(show_mesh_color);
    viewer.data.set_face_based(true);
    viewer.core.orthographic = true; //set to ortho view
    viewer.core.show_lines = false;
    viewer.launch();
}


//

//


#endif //PART_MATCH_VISUALIZE_H
