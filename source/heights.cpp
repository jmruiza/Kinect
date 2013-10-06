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
              << "\n  -ref  - Get height using the bigger distance as reference"
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
/*
int main (int argc, char** argv){

    // Check number of parameters
    if(argc < 2){
        std::cout << " Error: This program needs at least 1 parameter to work" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    Heights height(argv[1]);

    for(int i=2; i<argc; i++){
        if( strcmp(argv[i], "-cm") == 0 )
            height.distanceInMeters(false);
        else if( strcmp(argv[i], "-ref") == 0 )
            height.distanceAbsolute(false);
        else if( strcmp(argv[i], "-nf") == 0 )
            height.setNoFilter(true);
        else if( strcmp(argv[i], "-demo") == 0 )
            height.setDemoMode(true);
        else{
            std::cout << " Error: \"" << argv[i] << "\" Isn't a valid parameter" << std::endl;
            printUsage(argv[0]);
            return (1);
        }
    }

    height.run();
    return 0;
}
*/
// Debug mode
int main (){
    Heights height("../../../data/2013-09-27_135220");
    height.distanceInMeters(false);
    height.distanceAbsolute(false);
    height.setNoFilter(true);
    height.setDemoMode(true);
    height.run();

    return 0;
}

