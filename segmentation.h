#ifndef PART_MATCH_SEGMENTATION_H
#define PART_MATCH_SEGMENTATION_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/mesh_segmentation.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <igl/readOBJ.h>
#include <igl/copyleft/cgal/mesh_boolean.h>

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SM;
typedef boost::graph_traits<SM>::face_descriptor face_descriptor;

int segmentation(string filename, int f_id, int num_soft_cluster, float smooth, vector<Eigen::MatrixXd> &segments_v, vector<Eigen::MatrixXi> &segments_f)
{

    size_t found = filename.find_last_of("/");
    found = filename.substr(0,found).find_last_of("/");
    string dic =  filename.substr(0, found) + "/segmentation";


    SM mesh;
    std::ifstream cactus(filename);
    if ( !cactus || !(cactus >> mesh)) {
        std::cerr << "Input is not a triangle mesh" << std::endl;
        return EXIT_FAILURE;
    }
    //cactus >> mesh;

    typedef SM::Property_map<face_descriptor, double> Facet_double_map;
    Facet_double_map sdf_property_map;
    sdf_property_map = mesh.add_property_map<face_descriptor,double>("f:sdf").first;
    CGAL::sdf_values(mesh, sdf_property_map);
    // create a property-map for segment-ids
    typedef SM::Property_map<face_descriptor, std::size_t> Facet_int_map;
    Facet_int_map segment_property_map = mesh.add_property_map<face_descriptor,std::size_t>("f:sid").first;;
    // segment the mesh using default parameters for number of levels, and smoothing lambda
    // Any other scalar values can be used instead of using SDF values computed using the CGAL function
    std::size_t number_of_segments = CGAL::segmentation_from_sdf_values(mesh, sdf_property_map, segment_property_map,num_soft_cluster,smooth);
    typedef CGAL::Face_filtered_graph<SM> Filtered_graph;
    //print area of each segment and then put it in a Mesh and print it in an OFF file
    Filtered_graph segment_mesh(mesh, 0, segment_property_map);
    for(std::size_t id = 0; id < number_of_segments; ++id)
    {
        if(id > 0)
            segment_mesh.set_selected_faces(id, segment_property_map);
        std::cout << "Segment "<<id<<"'s area is : "<<CGAL::Polygon_mesh_processing::area(segment_mesh)<<std::endl;
        SM out;
        CGAL::copy_face_graph(segment_mesh, out);
        std::ostringstream oss;
        if(f_id!=-1)
            oss << dic <<"/male"<< f_id << "/Segment_" << id<<".off";
        else
            oss << dic << "/female" << "/Segment_" << id<<".off";
        std::ofstream os(oss.str().data());
        os<<out;
    }

    for(std::size_t id = 0; id < number_of_segments; ++id)
    {
        ostringstream oss;
        if(f_id != -1)
            oss << dic <<"/male"<< f_id << "/Segment_" << id<<".off";
        else
            oss << dic <<"/female" << "/Segment_" << id<<".off";
        Eigen::MatrixXd male_v; //vertex data
        Eigen::MatrixXi male_f; //face index data
        igl::readOFF(oss.str(), male_v, male_f);
        segments_v.push_back(male_v);
        segments_f.push_back(male_f);
    }

    int tmp = segments_v.size();

    if(f_id != -1){
        int size = segments_v.size();
        for(int i = 0; i<size;++i){
            for(int j = i; j<size;++j){
                if(j>i){
                    Eigen::MatrixXd show_mesh_v(segments_v[i].rows() + segments_v[j].rows(), segments_v[i].cols());
                    show_mesh_v << segments_v[i], segments_v[j];

                    Eigen::MatrixXi show_mesh_f(segments_f[i].rows() + segments_f[j].rows(), segments_f[i].cols());
                    show_mesh_f << segments_f[i], (segments_f[j].array() + segments_v[i].rows());


                    segments_v.push_back(show_mesh_v);
                    segments_f.push_back(show_mesh_f);
                } else
                    continue;
            }
        }
    }

    std::cout<<segments_v.size()<<std::endl;

    return tmp;
}

#endif //PART_MATCH_SEGMENTATION_H
