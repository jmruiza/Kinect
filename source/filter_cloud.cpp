#include <iostream>

int
main (int argc, char** argv)
{
    // Check number of parameters
    std::cout << argc << std::endl;
    if(argc < 2 || argc > 2){
        std::cout << "\n Usage: "<< argv[0] << "  point_cloud_file.pcd" <<"\n"
                  << "---------------------------------------------------------\n"
                  << " Press:\n"
                  << "  - ENTER    - to save the filtered Point Cloud\n"
                  << "  - Q        - to close the viewer and exit the program\n"
                  << "  - 1 - 9    - to apply a filter  \n"
                  << "---------------------------------------------------------\n"
                  << std::endl;
        return 1;
    }

      return (0);
}
