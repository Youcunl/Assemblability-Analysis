//a point cloud sampler for triangulated surface

#ifndef SAMPLER_H
#define SAMPLER_H

#include <math.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Sampler
{
  public:
    Eigen::MatrixXd *tri_v;
    //Eigen::MatrixXd *tri_vn;
    Eigen::MatrixXi *tri_f;
    int num_tris;
    double sum_area;
    double *weights;

    Sampler(Eigen::MatrixXd *tri_v, Eigen::MatrixXi *tri_f)
    {
        this->tri_v = tri_v;
        //this->tri_vn = tri_vn;
        this->tri_f = tri_f;
        this->num_tris = tri_f->rows();
        this->sum_area = 0;
        this->weights = new double[num_tris];
        //std::cout << this->tri_v << std::endl; //address
        //std::cout << (*this->tri_v).size() << std::endl; //real content
        //std::cout << this->tri_v->size() << std::endl; //real content
        //std::cout << "num_tris: " << num_tris << std::endl;
        //std::cout << "num_tris: " << (*this->tri_v).rows() << std::endl;
        //std::cout << "num_tris: " << this->tri_v->rows() << std::endl;
        //std::cout << "num_tris: " << tri_v->rows() << std::endl;

        for (int i = 0; i < num_tris; i++)
        {
            //std::cout << i << " " << ((*this->tri_f).row(i)) << std::endl;
            double tri_area = getTriArea(((*this->tri_v).row((*this->tri_f)(i, 0))), ((*this->tri_v).row((*this->tri_f)(i, 1))), ((*this->tri_v).row((*this->tri_f)(i, 2))));
            weights[i] = tri_area;
            sum_area += tri_area;
        }

        for (int i = 0; i < num_tris; i++)
        {
            weights[i] /= sum_area;
        }
        std::cout << "modela surface area: " << sum_area << std::endl;
    }

    //get the area of a triangle
    double getTriArea(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c)
    {
        double e0 = (b - a).norm();
        double e1 = (c - b).norm();
        double e2 = (a - c).norm();
        double s = (e0 + e1 + e2) / 2;
        double area = std::sqrt(s * (s - e0) * (s - e1) * (s - e2));
        return area;
    }

    //pick a random point on the triangle
    void getRandomPtOnTri(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c, Eigen::Vector3d &out_pt)
    {
        double tri_area = getTriArea(a, b, c);
        double r0 = ((double)std::rand() / (RAND_MAX));
        double r1 = ((double)std::rand() / (RAND_MAX));
        Eigen::Vector3d e0 = b - a;
        Eigen::Vector3d e1 = c - a;
        Eigen::Vector3d pt = a + r0 * e0 + r1 * e1;
        //is pt in abc?
        //http://blackpawn.com/texts/pointinpoly/
        //if not r0=1-r0, r1=1-r1
        Eigen::Vector3d e2 = pt - a;
        double dot00 = e0.dot(e0); //dot(e0, e0);
        double dot01 = e0.dot(e1); //dot(e0, e1);
        double dot02 = e0.dot(e2); //dot(e0, e2);
        double dot11 = e1.dot(e1); //dot(e1, e1);
        double dot12 = e1.dot(e2); //dot(e1, e2);
        double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        if ((u >= 0) && (v >= 0) && (u + v <= 1))
        {
            out_pt = pt;
        }
        else
        {
            r0 = 1.0 - r0;
            r1 = 1.0 - r1;
            pt = a + r0 * e0 + r1 * e1;
            e2 = pt - a;
            double dot00 = e0.dot(e0); //dot(e0, e0);
            double dot01 = e0.dot(e1); //dot(e0, e1);
            double dot02 = e0.dot(e2); //dot(e0, e2);
            double dot11 = e1.dot(e1); //dot(e1, e1);
            double dot12 = e1.dot(e2); //dot(e1, e2);
            invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            if ((u >= 0) && (v >= 0) && (u + v <= 1))
            {
                out_pt = pt;
            }
            else
            {
                //should never be executed
                std::cout << "u: " << u << ", v: " << v << std::endl;
                std::cout << "r0: " << r0 << ", r1: " << r1 << std::endl;
                std::cout << "pt: " << pt << std::endl;
                std::cout << "a: " << a << ", b: " << b << ", c: " << c << std::endl;
                std::cout << "wrong implementation" << std::endl;
                while (true)
                {
                    ;
                }
            }
        }
    }

    //sample the mesh to point cloud
    pcl::PointCloud<pcl::PointXYZ> getPointCloud(int sample_density)
    {
        int num_samples = int(sample_density * sum_area + 0.5);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = num_samples;
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize(num_samples);

        for (int i = 0; i < num_samples; i++)
        {
            //for every sample, randomly choose a tri
            int tri_index = 0;
            double x = ((double)std::rand() / (RAND_MAX));
            for (int j = 0; j < num_samples; j++)
            {
                if (x <= weights[j])
                {
                    tri_index = j;
                    break;
                }
                x -= weights[j];
            }

            //for a random tri, randomly select a point

            Eigen::Vector3d pt;
            getRandomPtOnTri(((*this->tri_v).row((*this->tri_f)(tri_index, 0))), ((*this->tri_v).row((*this->tri_f)(tri_index, 1))), ((*this->tri_v).row((*this->tri_f)(tri_index, 2))), pt);
            //write to cloud
            cloud.points[i].x = pt(0);
            cloud.points[i].y = pt(1);
            cloud.points[i].z = pt(2);
        }

        return cloud;
    }
};

#endif