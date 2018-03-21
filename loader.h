#ifndef PART_MATCH_LOADER_H
#define PART_MATCH_LOADER_H

#include <iostream>
#include <string>
#include <vector>
#include <igl/readOBJ.h>

#include <fstream>
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <sys/types.h>

#include <igl/viewer/Viewer.h>

using namespace std;

class Loader{
public:
    string workspace;
    int num_males;
private:
    Eigen::MatrixXd female_v; //vertex data
    Eigen::MatrixXi female_f;
    Eigen::MatrixXd female_vn;
    vector<Eigen::MatrixXd> males_v;
    vector<Eigen::MatrixXi> males_f;
    vector<Eigen::MatrixXd> males_vn;

public:
    Loader(string workspace, int num_males){
        this->workspace = workspace;
        this->num_males = num_males;
        check();
        load();
    }
    void get_female(Eigen::MatrixXd &female_v, Eigen::MatrixXi &female_f, Eigen::MatrixXd &female_vn){
        female_v = this->female_v;
        female_f = this->female_f;
        female_vn = this->female_vn;

    }
    void get_males(vector<Eigen::MatrixXd> &males_v, vector<Eigen::MatrixXi> &males_f, vector<Eigen::MatrixXd> &males_vn){
        males_v = this->males_v;
        males_f = this->males_f;
        males_vn = this->males_vn;
    }

private:
    int check(){
        //check female
        fstream fs1;
        ostringstream female_name;
        female_name<<workspace<<"/female.obj";
        fs1.open(female_name.str(), ios::in);
        if(fs1){
            cout << female_name.str()<<" exist." << endl;
            fs1.close();
        }else{
            cout<<female_name.str()<<" not exist."<<endl;
            fs1.close();
            return -1;
        }

        //check males
        for(int i = 0;i<num_males;++i){
            fstream fs2;
            ostringstream male_name;
            male_name<<workspace<<"/male"<<i<<".obj";
            fs2.open(male_name.str(), ios::in);
            if (fs2){
                cout<<male_name.str()<<" exist."<<endl;
                fs2.close();
            }else {
                cout <<male_name.str()<< " not exist." << endl;
                fs2.close();
                return -1;
            }
        }

        //check folders
        vector<string> folder_names{"/pcd", "/segmentation","/tmp_off"};
        for(int i =0; i<folder_names.size();++i){
            boost::filesystem::path folder(workspace+folder_names[i]);
            if(boost::filesystem::exists(folder) && boost::filesystem::is_directory(folder))
                cout<<workspace+folder_names[i]<<" exsit"<<endl;
            else {
                mkdir((workspace + folder_names[i]).c_str(), S_IRWXU);
                cout << workspace + folder_names[i] << " created" << endl;
            }

            ostringstream tmp_f;
            tmp_f<<workspace<<folder_names[i]<<"/female";
            boost::filesystem::path tmp_f_folder(tmp_f.str());
            if(boost::filesystem::exists(tmp_f_folder) && boost::filesystem::is_directory(tmp_f_folder))
                cout<<tmp_f.str()<<" exsit"<<endl;
            else{
                mkdir(tmp_f.str().c_str(), S_IRWXU);
                cout<<tmp_f.str()<<" created"<<endl;
            }
            for(int j =0;j<num_males;++j){
                ostringstream tmp;
                tmp<<workspace<<folder_names[i]<<"/male"<<j;
                boost::filesystem::path tmp_folder(tmp.str());
                if(boost::filesystem::exists(tmp_folder) && boost::filesystem::is_directory(tmp_folder))
                    cout<<tmp.str()<<" exsit"<<endl;
                else{
                    mkdir(tmp.str().c_str(), S_IRWXU);
                    cout<<tmp.str()<<" created"<<endl;
                }
            }

            if(folder_names[i]=="/pcd"){
                ostringstream tmp_ff;
                tmp_ff<<workspace<<"/pcd/vertex";
                boost::filesystem::path tmp_ff_folder(tmp_ff.str());
                if(boost::filesystem::exists(tmp_ff_folder) && boost::filesystem::is_directory(tmp_ff_folder))
                    cout<<tmp_ff.str()<<" exsit"<<endl;
                else{
                    mkdir(tmp_ff.str().c_str(), S_IRWXU);
                    cout<<tmp_ff.str()<<" created"<<endl;
                }
            }

        }

        boost::filesystem::path folder(workspace+"/ret");
        if(boost::filesystem::exists(folder) && boost::filesystem::is_directory(folder))
            cout<<workspace+"/ret"<<" exsit"<<endl;
        else {
            mkdir((workspace+"/ret").c_str(), S_IRWXU);
            cout << workspace+"/ret" << " created" << endl;
        }

        return 0;
    }
    int load(){
        //load female
        ostringstream female_name;
        female_name<<workspace<<"/female.obj";
        Eigen::MatrixXd female_tc;
        Eigen::MatrixXi female_ftc;
        Eigen::MatrixXi female_fn;
        igl::readOBJ(female_name.str(), female_v, female_tc, female_vn, female_f, female_ftc, female_fn);

        //load male
        for(int i =0;i<num_males;++i){
            ostringstream male_name;
            male_name<<workspace<<"/male"<<i<<".obj";
            Eigen::MatrixXd male_v; //vertex data
            Eigen::MatrixXi male_f;
            Eigen::MatrixXd male_vn;
            Eigen::MatrixXd male_tc;
            Eigen::MatrixXi male_ftc;
            Eigen::MatrixXi male_fn;
            igl::readOBJ(male_name.str(), male_v, male_tc, male_vn, male_f, male_ftc, male_fn);
            males_v.push_back(male_v);
            males_f.push_back(male_f);
            males_vn.push_back(male_vn);
        }
        return 0;
    }
};





#endif //PART_MATCH_LOADER_H
