
#include "tensorflow_voxelgrid/voxelgrid.h"
#include <iostream>

//
// Created by bradsaund on 11/24/20.
//


void Voxelgrid::test() {
    std::cout << "hi\n";
}


int main(){
    Voxelgrid v;
    v.test();
    v.otherTest();
    v.testTensor();
    return 0;
}
