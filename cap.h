#ifndef PART_MATCH_CAP_H
#define PART_MATCH_CAP_H

bool isCoplanar(Eigen::Vector3i f1, Eigen::MatrixXd v1, Eigen::Vector3i f2, Eigen::MatrixXd v2)
{
    Eigen::Vector3d a1(v1.row(f1(0)));
    Eigen::Vector3d b1(v1.row(f1(1)));
    Eigen::Vector3d c1(v1.row(f1(2)));

    Eigen::Vector3d a2(v2.row(f2(0)));
    Eigen::Vector3d b2(v2.row(f2(1)));
    Eigen::Vector3d c2(v2.row(f2(2)));

    double d1 = (a2 - a1).dot((b1 - a1).cross(c1 - a1));
    double d2 = (b2 - a1).dot((b1 - a1).cross(c1 - a1));
    double d3 = (c2 - a1).dot((b1 - a1).cross(c1 - a1));

    if (d1 == 0 && d2 == 0 && d3 == 0)
    {
        //std::cout << "coplanar" << std::endl;
        return true;
    }
    return false;
}

void move_cap(Eigen::MatrixXd core_v, Eigen::MatrixXi core_f, Eigen::MatrixXd female_ch_v, Eigen::MatrixXi female_ch_f,
              Eigen::MatrixXi &core_nocap_f){
    for (int i = 0; i < core_f.rows(); i++)
    {
        bool coplanar = false;
        for (int j = 0; j < female_ch_f.rows(); j++)
        {
            Eigen::Vector3i f1(core_f.row(i));
            Eigen::Vector3i f2(female_ch_f.row(j));
            coplanar = isCoplanar(f1, core_v, f2, female_ch_v) || coplanar;
        }
        if (!coplanar)
        {
            core_nocap_f.conservativeResize(core_nocap_f.rows() + 1, 3);
            core_nocap_f.row(core_nocap_f.rows() - 1) = core_f.row(i);
        }
    }
}

void move_cap2(Eigen::MatrixXd core_v, Eigen::MatrixXi core_f, Eigen::MatrixXd female_ch_v, Eigen::MatrixXi female_ch_f,
              Eigen::MatrixXi &core_nocap_f){
    for (int i = 0; i < core_f.rows(); i++)
    {
        bool coplanar = false;
        for (int j = 0; j < female_ch_f.rows(); j++)
        {
            Eigen::Vector3i f1(core_f.row(i));
            Eigen::Vector3i f2(female_ch_f.row(j));
            coplanar = isCoplanar(f1, core_v, f2, female_ch_v) || coplanar;
        }
        if (coplanar)
        {
            core_nocap_f.conservativeResize(core_nocap_f.rows() + 1, 3);
            core_nocap_f.row(core_nocap_f.rows() - 1) = core_f.row(i);
        }
    }
}

#endif //PART_MATCH_CAP_H
