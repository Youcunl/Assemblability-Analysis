#ifndef PART_MATCH_COST_H
#define PART_MATCH_COST_H

#include "igl/volume.h"
#include <igl/copyleft/cgal/mesh_boolean.h>

double cal_area(Eigen::MatrixXd V, Eigen::MatrixXi F){
    Eigen::MatrixXd V2(V.rows() + 1, V.cols());
    V2.topRows(V.rows()) = V;
    V2.bottomRows(1).setZero();
    Eigen::MatrixXi T(F.rows(), 4);
    T.leftCols(3) = F;
    T.rightCols(1).setConstant(V.rows());
    Eigen::VectorXd vol;
    igl::volume(V2, T, vol);
    return std::abs(vol.sum());
}

double cal_cost(Eigen::MatrixXd a_v, Eigen::MatrixXi a_f,
                Eigen::MatrixXd b_v, Eigen::MatrixXi b_f,
                Eigen::MatrixXd c_v, Eigen::MatrixXi c_f){

    Eigen::MatrixXd i1_v, i2_v;
    Eigen::MatrixXi i1_f, i2_f, J;
    igl::copyleft::cgal::mesh_boolean(a_v, a_f, b_v, b_f, "intersect", i1_v, i1_f, J);
    igl::copyleft::cgal::mesh_boolean(a_v, a_f, c_v, c_f, "intersect", i2_v, i2_f, J);

    return cal_area(i1_v, i1_f)/cal_area(i2_v, i2_f);
}

#endif //PART_MATCH_COST_H
