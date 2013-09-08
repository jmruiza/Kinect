#include <iostream>
#include "filters_viewer.h"

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << "  point_cloud_file.pcd -filter\n"
              << "---------------------------------------------------------\n"
              << " Press:\n"
              << "  - ENTER    - to save the filtered Point Cloud\n"
              << "  - Q        - to close the viewer and exit the program\n\n"
              << " Filters List:\n"
              << "  - -SOR     - to apply a Statistical Outlier Removal filter\n"
              << "  - -ROR     - to apply a Radius Outlier Removal filter\n"
              << "  - -CR      - to apply a Conditional Removal filter\n"
              << "  - -VG      - to apply a Voxel Grid filter\n"
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
        return (1);
    }

    // Check the filename (First Param):
    std::string parameter = argv[1];
    size_t position_ext = parameter.find(".pcd");
    std::string extension;
    std::string filename;

    // Check that filename has a point
    if( position_ext == parameter.npos ){
        std::cout << " Error: The given parameter \"" << parameter << "\" must have a valid file name!" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    extension = parameter.substr(position_ext+1, parameter.length());
    filename = parameter.substr(0, position_ext);

    // Check that file name has a "*.pcd" extension
    if( extension.compare("pcd") ){
        std::cout << " Error: The given parameter \"" << parameter << "\" isn't a \"*.pcd\" file!" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    // Check selected filter (Second Parameter)
    std::string filter = argv[2];

//    std::cout << "Input Parameter: " << parameter << std::endl;
//    std::cout << "Filename: " << filename << std::endl;
//    std::cout << "Extension: " << extension << std::endl;
//    std::cout << "Filter: " << filter << std::endl;

    FiltersViewer fv;
    fv.set_FileNames(filename);
    fv.load_Cloud();

    // -SOR - Statistical Outlier Removal filter
    if(!filter.compare("-SOR")){
        std::cout << " -SOR -> Statistical Outlier Removal filter" << std::endl;
        fv.filter_StatisticalOutlierRemoval();
        fv.run();
        return (0);
    }

    // -ROR - Radius Outlier Removal filter
    if(!filter.compare("-ROR")){
        std::cout << " -ROR -> Radius Outlier Removal filter" << std::endl;
        std::cout << " Please wait.." << std::endl;
        fv.filter_RadiusOutlierRemoval();
        fv.run();
        return (0);
    }

    // -CR - Conditional Removal filter
    if(!filter.compare("-CR")){
        std::cout << " -CR -> Conditional Removal filter" << std::endl;
        std::cout << " Please wait.." << std::endl;
        fv.filter_ConditionalRemoval();
        fv.run();
        return (0);
    }

    // -VG - Voxel Grid filter
    if(!filter.compare("-VG")){
        std::cout << " -VG -> Voxel Grid filter" << std::endl;
        std::cout << " Please wait.." << std::endl;
        fv.filter_VoxelGrid();
        fv.run();
        return (0);
    }

    // If the selected filter isn't valid
    std::cout << " Error: \"" << filter << "\" isn't valid filter" << std::endl;
    printUsage(argv[0]);
    return (1);
}
