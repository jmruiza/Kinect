#include <iostream>
#include "heights.h"

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << " filename \n"
              << std::endl;
}

int main (int argc, char** argv){
    // Number of parameters needed
    int n_params = 1;

    // Check number of parameters
    if(argc < n_params+1 || argc > n_params+1){
        std::cout << " Error: This program needs " << n_params << " parameter(s) to work" << std::endl;
        printUsage(argv[0]);
        return (1);
    }

    // Set the filename
    Heights height(argv[1]);

    return 0;
}

