#include <iostream>
#include "filters_viewer.h"

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << "  point_cloud_file.pcd -filter\n"
              << "---------------------------------------------------------\n"
              << " Press:\n"
              << "  - ENTER    - to save the filtered Point Cloud\n"
              << "  - Q        - to close the viewer and exit the program\n"
              << " Filters List:\n"
              << "  - -SOR     - to apply a Statistical Outlier Removal filter\n"
              << "---------------------------------------------------------\n"
              << std::endl;
}

int main (int argc, char** argv){

    // Number of parameters needed
    int n_params = 2;

    // Check number of parameters
    if(argc < n_params+1 || argc > n_params+1){
        std::cout << " Error: This program needs " << n_params << " parameter(s) to work" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Check the filename:
    std::string parameter = argv[1];
    size_t position_ext = parameter.find(".pcd");
    std::string extension;
    std::string filename;

    // Check that filename has a point
    if( position_ext == parameter.npos ){
        std::cout << " Error: The given parameter \"" << parameter << "\" must have a valid file name!" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    extension = parameter.substr(position_ext+1, parameter.length());
    filename = parameter.substr(0, position_ext);

    // Check that file name has a "*.pcd" extension
    if( extension.compare("pcd") ){
        std::cout << " Error: The given parameter \"" << parameter << "\" isn't a \"*.pcd\" file!" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

//    std::cout << "Input Parameter: " << parameter << std::endl;
//    std::cout << "Filename: " << filename << std::endl;
//    std::cout << "Extension: " << extension << std::endl;

//    if()
//    {
        FiltersViewer fv;
        fv.set_FileNames(filename);
        fv.load_Cloud();
        fv.fil_StatisticalOutlierRemoval();
        fv.run();
        return (0);
//    }

//    return (0);
}
