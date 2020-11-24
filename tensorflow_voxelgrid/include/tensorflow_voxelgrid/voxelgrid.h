//
// Created by bradsaund on 11/24/20.
//

#ifndef TENSORFLOW_VOXELGRID_VOXELGRID_H
#define TENSORFLOW_VOXELGRID_VOXELGRID_H


#include <iostream>
#include <tensorflow/c/c_api.h>

class Voxelgrid {
public:
    void test();
    static void otherTest()
    {
        std::cout << "other test\n";
    }
};

#endif //TENSORFLOW_VOXELGRID_VOXELGRID_H
