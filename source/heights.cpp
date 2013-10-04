#include <iostream>
#include "heights.h"

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << " [filenames] [options]"
              << "\n ------------------------------------------------------------------"
              << "\n filenames: The pcd and jpg files must have the same except for extension"
              << "\n            Example: For \"file.pcd\" and \"file.jpg\" you must enter \"file\""
              << "\n Options:"
              << "\n  -cm   - Get distance or height in centimeters"
              << "\n          (By default is given in meters)"
              << "\n  -abs  - Get height using the bigger distance as reference"
              << "\n          (By default is given the absolute distance)"
              << "\n  -nf   - Not filter data, use the data directly from pcd file"
              << "\n  -demo - Demo mode, the user selects some points and get the"
              << "\n          heights or distances "
              << "\n Controls:"
              << "\n    r           - Reset captured points (Demo mode)"
              << "\n    ENTER       - Get the heights or distances of the captured points(Demo mode)"
              << "\n    ESC or Q    - Finish and close program"
              << std::endl;
}

int main (int argc, char** argv){

    // Check number of parameters
    if(argc < 2 || argc > 4){
        std::cout << " Error: This program needs at least 1 parameter to work" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    if(argc == 2){
        Heights height(argv[1]);
        height.run();
    }

    if(argc == 3){
        std::string unit = argv[2];
        Heights height(argv[1]);
        if(!unit.compare("false")){
            height.distanceInMeters(false);
        }
        height.run();
    }

    if(argc == 4){
        std::string unit = argv[2];
        std::string absolute = argv[3];
        Heights height(argv[1]);
        if(!unit.compare("false"))
            height.distanceInMeters(false);
        if(!absolute.compare("false"))
            height.distanceAbsolute(false);
        height.run();    }

    return 0;
}

/* // Debug mode
int main (){
    Heights height("../../../data/2013-09-27_135220");
    height.run();
    return 0;
}
*/
