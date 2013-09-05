#ifndef FILTERS_VIEWER_H
#define FILTERS_VIEWER_H

#include <iostream>


class FiltersViewer{
public:
    std::string cloud_in;
    std::string cloud_out;

    // Constructor
    FiltersViewer( std::string cloud_in){
        std::stringbuf tmp;
        tmp << cloud_in << "filtered.pcd";
        cloud_out = tmp.str();

        std::cout << "Input file: " << cloud_in << "\n"
                  << "Output file: " << cloud_out << std::endl;

    }

private:




};

#endif // FILTERS_VIEWER_H
