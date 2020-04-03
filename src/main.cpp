#include <iostream>
#include <iomanip>
#include <fstream>
#include "slamBase.h"

using namespace std;

int main(int argc, char** argv)
{
    std::cout<< "hello SLAM" << std::endl;

    ImgReader imgRd;
    DepthReader depthRd;

    for(int i=0; i<500; i++)
    {
        cout << endl;

        cout << "img_string: " << imgRd.getImg(i) << endl;
        cout << "depth_string: " << depthRd.getDepth(i) << endl;

        long double img_index = stold(imgRd.getImg(i));
        long double depth_index = stold(depthRd.getDepth(i));

        cout << "img_index: " << std::setprecision(20) << img_index << endl;
        cout << "depth_index: " << std::setprecision(20) << depth_index << endl;

        cout << endl;
    }

    return 0;
}

