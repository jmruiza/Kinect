#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>


void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << "  point_cloud_file.pcd" <<"\n"
              << "---------------------------------------------------------\n"
              << " Press:\n"
              << "  - ENTER    - to save the filtered Point Cloud\n"
              << "  - Q        - to close the viewer and exit the program\n"
              << "  - 1 - 9    - to apply a filter  \n"
              << "---------------------------------------------------------\n"
              << std::endl;
}

int main (int argc, char** argv){
    // Check number of parameters
    if(argc < 2 || argc > 2){
        std::cout << " Error: This program needs a parameter to work " << std::endl;
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

    std::cout << "Input Parameter: " << parameter << std::endl;
    std::cout << "Filename: " << filename << std::endl;
    std::cout << "Extension: " << extension << std::endl;

    // Functions for filters
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Read the input cloud data
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> (parameter, *cloud_in);


    return (0);
}
