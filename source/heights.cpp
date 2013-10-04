#include <iostream>
#include "heights.h"

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << " [file name] [unit of measure] [absolute measure]"
              << "\n Example: ./Heights ../../filename true true"
              << "\n ------------------------------------------------------------------"
              << "\n unit of measure    -   meters (true or null)"
              << "\n                    -   centimeters (false)"
              << "\n absolute measure   -   absolute (true or null) "
              << "\n                    -   relative (false)"
              << std::endl;
}

int main (int argc, char** argv){

    // Check number of parameters
    if(argc < 2 || argc > 4){
        std::cout << " Error: This program needs 1 or 3 parameter(s) to work" << std::endl;
        std::cout << argc << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    if(argc == 2){
        Heights height(argv[1]);
        height.run();
    }

    if(argc == 3){
        Heights height(argv[1]);
        height.distanceInMeters(argv[2]);
        height.run();
    }

    if(argc == 4){
        Heights height(argv[1]);
        height.distanceInMeters(argv[2]);
        height.distanceAbsolute(argv[3]);
        height.run();
    }

    return 0;
}

/* // Debug mode
int main (){
    Heights height("../../../data/2013-09-27_135220");
    height.run();
    return 0;
}
*/
