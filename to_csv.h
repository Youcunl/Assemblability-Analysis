#ifndef PART_MATCH_EIGEN2CSV_H
#define PART_MATCH_EIGEN2CSV_H

#include <iostream>
#include <fstream>

using namespace std;

void writeToCSV(string name, Eigen::MatrixXd matrix)
{
    ofstream file((name+".csv").c_str());

    for(int  i = 0; i < matrix.rows(); i++){
        for(int j = 0; j < matrix.cols(); j++){
            if(j+1 == matrix.cols()){
                file<<matrix(i,j);
            }else{
                file<<matrix(i,j)<<',';
            }
        }
        file<<'\n';
    }

    file.close();
}

void writeToCSV(string name, vector<vector<Eigen::Matrix4d>> matrices) {
    for (int n = 0; n < matrices.size(); ++n) {
        for (int m = 0; m < matrices[n].size(); ++m) {
            writeToCSV(name + "_" + to_string(n) + "_" + to_string(m), matrices[n][m]);
        }
    }
}


void readFromCSV(string path, Eigen::MatrixXd &costs, vector<vector<Eigen::Matrix4d>> &index) {

    std::ifstream in(path+"/costs.csv");
    std::string line;
    int row = 0;
    int col = 0;

    if (in.is_open()) {
        while (std::getline(in, line)) {
            char *ptr = (char *) line.c_str();
            int len = line.length();

            col = 0;

            char *start = ptr;
            for (int i = 0; i < len; i++) {

                if (ptr[i] == ',') {
                    costs(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            costs(row, col) = atof(start);

            row++;
        }

        in.close();
    }

    for(int i = 0;i<4;++i){
        vector<Eigen::Matrix4d> tmpVec;
        for(int j = 0;j<4;++j){
            std::ifstream in(path+"/matrix_"+to_string(i)+"_"+to_string(j)+".csv");
            std::string line;
            int row = 0;
            int col = 0;

            Eigen::Matrix4d tmp;
            if (in.is_open()) {
                while (std::getline(in, line)) {
                    char *ptr = (char *) line.c_str();
                    int len = line.length();

                    col = 0;

                    char *start = ptr;
                    for (int i = 0; i < len; i++) {

                        if (ptr[i] == ',') {
                            tmp(row, col++) = atof(start);
                            start = ptr + i + 1;
                        }
                    }
                    tmp(row, col) = atof(start);

                    row++;
                }

                in.close();
            }
            tmpVec.push_back(tmp);
        }
        index.push_back(tmpVec);
    }

}

#endif //PART_MATCH_EIGEN2CSV_H
