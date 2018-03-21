#include <pcl/io/pcd_io.h>
#include <string>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h>

inline double
uniform_deviate (int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     Eigen::Vector4f& p)
{
    float r1 = static_cast<float> (uniform_deviate (rand ()));
    float r2 = static_cast<float> (uniform_deviate (rand ()));
    float r1sqr = std::sqrt (r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
    p[3] = 0;
}

inline void
randPSurface (Eigen::MatrixXd v, Eigen::MatrixXi f,
              std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p,
              bool calcNormal, Eigen::Vector3f& n)
{
    float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

    std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
    int el = int(low - cumulativeAreas->begin ());

    double A[3], B[3], C[3];

    A[0] = v(f(el,0),0);
    A[1] = v(f(el,0),1);
    A[2] = v(f(el,0),2);
    B[0] = v(f(el,1),0);
    B[1] = v(f(el,1),1);
    B[2] = v(f(el,1),2);
    C[0] = v(f(el,2),0);
    C[1] = v(f(el,2),1);
    C[2] = v(f(el,2),2);

    if (calcNormal)
    {
        // OBJ: Vertices are stored in a counter-clockwise order by default
        Eigen::Vector3f v1 = Eigen::Vector3f (A[0], A[1], A[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
        Eigen::Vector3f v2 = Eigen::Vector3f (B[0], B[1], B[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
        n = v1.cross (v2);
        n.normalize ();
    }

    randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                         float (B[0]), float (B[1]), float (B[2]),
                         float (C[0]), float (C[1]), float (C[2]), p);
}

double ComputeArea(double v1[3], double v2[3], double v3[3])
{
    double ax, ay, az, bx, by, bz;

    // order is important!!! maintain consistency with triangle vertex order
    ax = v3[0] - v2[0]; ay = v3[1] - v2[1]; az = v3[2] - v2[2];
    bx = v1[0] - v2[0]; by = v1[1] - v2[1]; bz = v1[2] - v2[2];

    double n[3];
    n[0] = (ay * bz - az * by);
    n[1] = (az * bx - ax * bz);
    n[2] = (ax * by - ay * bx);

    return 0.5*sqrt( n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
}

void
uniform_sampling (Eigen::MatrixXd v, Eigen::MatrixXi f, bool calcNormal, size_t density, pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud_out)
{
    double p1[3], p2[3], p3[3], totalArea = 0;
    std::vector<double> cumulativeAreas (f.rows(), 0);

    for (int i = 0; i<f.rows(); ++i)
    {
        p1[0] = v(f(i,0),0);
        p1[1] = v(f(i,0),1);
        p1[2] = v(f(i,0),2);
        p2[0] = v(f(i,1),0);
        p2[1] = v(f(i,1),1);
        p2[2] = v(f(i,1),2);
        p3[0] = v(f(i,2),0);
        p3[1] = v(f(i,2),1);
        p3[2] = v(f(i,2),2);
        totalArea += ComputeArea(p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }

    size_t n_samples = density*totalArea;
    cout<<n_samples<<endl;

    cloud_out.points.resize (n_samples);
    cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
    cloud_out.height = 1;

    for (int i = 0; i < n_samples; i++)
    {
        Eigen::Vector4f p;
        Eigen::Vector3f n;
        randPSurface (v, f, &cumulativeAreas, totalArea, p, calcNormal, n);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];


        if (calcNormal) {
            cloud_out.points[i].normal_x = n[0];
            cloud_out.points[i].normal_y = n[1];
            cloud_out.points[i].normal_z = n[2];
        }
    }
}

void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );

    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud ( cloud );

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud ( cloud );
    normalEstimator.setSearchMethod ( searchTree );
    normalEstimator.setKSearch ( 15 );
    normalEstimator.compute ( *normals );

    pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}

using namespace std;
using namespace pcl;
using namespace pcl::io;


/* ---[ */
pcl::PointCloud<pcl::PointXYZ>::Ptr meshSampling (Eigen::MatrixXd v, Eigen::MatrixXi f, int density)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    uniform_sampling (v, f, false, density, *cloud_1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    // Strip uninitialized normals from cloud:
    pcl::copyPointCloud (*cloud_1, *cloud_xyz);
    //savePCDFileASCII ("../models/pcd/female.pcd", *cloud_xyz);
    return cloud_xyz;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr meshSampling_n (Eigen::MatrixXd v, Eigen::MatrixXi f, int density)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    uniform_sampling (v, f, true, density, *cloud_1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    // Strip uninitialized normals from cloud:
    pcl::copyPointCloud (*cloud_1, *cloud_xyz);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    addNormal(cloud_xyz, cloud_with_normals);
    //savePCDFileASCII ("../models/pcd/female.pcd", *cloud_xyz);
    return cloud_with_normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vertex2cloud (Eigen::MatrixXd v)
{
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    // Fill in the cloud data
    cloud2.width    = v.rows();
    cloud2.height   = 1;
    cloud2.is_dense = false;
    cloud2.points.resize (cloud2.width * cloud2.height);

    for (size_t j = 0; j < cloud2.points.size (); ++j)
    {
        cloud2.points[j].x = v(j,0);
        cloud2.points[j].y = v(j,1);
        cloud2.points[j].z = v(j,2);
    }
    return cloud2.makeShared();
}