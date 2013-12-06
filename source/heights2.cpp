#include <iostream>
#include "heights2.h"

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << " [filenames] [options]"
              << "\n ------------------------------------------------------------------------"
              << "\n filenames: The pcd and jpg files must have the same except for extension"
              << "\n            Example: For \"file.pcd\" and \"file.jpg\" you must enter \"file\""
              << "\n Options:"
              << "\n  -cm   - Set centimeters as the unit of measure"
              << "\n          (By default is meters)"
              << "\n  -abs  - Get absolute distance to points"
              << "\n          (By default get the height using a reference))"
              << "\n  -nf   - Not filter data, use the data directly from pcd file"
              << "\n  -mp   - Multiple points mode, the user selects some points and get the"
              << "\n          heights or distances "
              << "\n  -cr   - Choose Reference mode, reference point is selected"
              << std::endl;
}

int main (int argc, char** argv){
printUsage(argv[0]);
    /*
    // Check number of parameters
    if(argc < 2){
        std::cout << " Error: This program needs at least 1 parameter to work" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    Heights height(argv[1]);

    for(int i=2; i<argc; i++){
        if( strcmp(argv[i], "-cm") == 0 )
            height.setDistanceInMeters(false);
        else if( strcmp(argv[i], "-ref") == 0 )
            height.setDistanceAbsolute(false);
        else if( strcmp(argv[i], "-nf") == 0 )
            height.setNoFilter(true);
        else if( strcmp(argv[i], "-mp") == 0 )
            height.setMPMode(true);
        else if( strcmp(argv[i], "-cr") == 0 )
            height.setChooseReference(true);
        else{
            std::cout << " Error: \"" << argv[i] << "\" Isn't a valid parameter" << std::endl;
            printUsage(argv[0]);
            return (1);
        }
    }

    height.setAdjustmentPixels(-20, 21);
//    height.pixelAdjust();
    height.run();
    */
    return 0;
}

/*// Debug mode
int main (){
    Heights height("../../../data/2013-09-27_135220");
    height.distanceInMeters(true);
    height.distanceAbsolute(true);
    height.setNoFilter(true);
    height.setDemoMode(false);
    height.run();

    return 0;
}
*/
