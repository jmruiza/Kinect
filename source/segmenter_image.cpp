#include <iostream>
#include "segmenter.h"

void printUsage(char* program_name){
    std::cout << "\n Usage: "<< program_name << "  image_file.jpg \n"
              << "---------------------------------------------------------\n"
              << " Press:\n"
              << "  - ENTER    - to save the segmented image\n"
              << "  - Q/ESCAPE - to close the viewer and exit the program\n"
              << "---------------------------------------------------------\n"
              << std::endl;
}

int main(int argc, char** argv){
    // Number of parameters needed
    int n_params = 1;

    // Check number of parameters
    if(argc < n_params+1 || argc > n_params+1){
        std::cout << " Error: This program needs " << n_params << " parameter(s) to work" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    // Check the filename (First Param):
    std::string parameter = argv[1];
    size_t position_ext = parameter.find(".jpg");
    std::string extension;
    std::string filename;

    // Check that filename has a point
    if( position_ext == parameter.npos ){
        std::cout << " Error: The given parameter \"" << parameter << "\" isn't a \"*.jpg\" file!" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    extension = parameter.substr(position_ext+1, parameter.length());
    filename = parameter.substr(0, position_ext);

//    std::cout << "Input Parameter: " << parameter << std::endl;
//    std::cout << "Filename: " << filename << std::endl;
//    std::cout << "Extension: " << extension << std::endl;

//    cv::Mat img = cv::imread(parameter);
//    cv::Mat segmented, binary;




    Segmenter seg(parameter);
    seg.run();
//    segmented = seg.get_image_segmented();

//    cv::namedWindow("Original");
//    cv::namedWindow("Segmented");

//    cv::moveWindow("Original", 0, 0);
//    cv::moveWindow("Segmented", 10, 0);

//    cv::imshow("Original", img);
//    cv::imshow("Segmented", segmented);

//    cv::waitKey();
    return (0);
}
