#include <iostream>
#include "heights.h"

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << " [file name] [unit of measure] [absolute measure]"
              << "\n Example: ./Heights ../../filename true true"
              << "\n ------------------------------------------------------------------"
              << std::endl;
}

int main (int argc, char** argv){
    // Number of parameters needed
    int n_params = 3;

    // Check number of parameters
    if(argc < n_params+1 || argc > n_params+1){
        std::cout << " Error: This program needs " << n_params << " parameter(s) to work" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    // Set the filename
    Heights height(argv[1]);
    height.distanceInMeters(false);
    height.distanceAbsolute(false);
    height.run();
}

/* // Debug mode
int main (){
    Heights height("../../../data/2013-09-27_135220");
    height.run();
    return 0;
}
*/
