#include <iostream>
#include "heights.h"

int main(){
    cv::Mat image;
    image = cv::imread("../../../data/2013-09-04_145744.jpg", 0);

    Heights heights(image);

    return 0;
}

