//
// Created by bradsaund on 11/24/20.
//

#ifndef TENSORFLOW_VOXELGRID_VOXELGRID_H
#define TENSORFLOW_VOXELGRID_VOXELGRID_H


#include <iostream>
#include <tensorflow/c/c_api.h>
#include <vector>
#include <numeric>
#include "tensorflow_voxelgrid/scope_guard.h"


static void DeallocateTensor(void* data, std::size_t, void*) {
    std::free(data);
    std::cout << "Deallocate tensor" << std::endl;
}

class Voxelgrid {
public:
    void test();
    static void otherTest()
    {
        std::cout << "other test\n";
        printf("Hello from TensorFlow C library version %s\n", TF_Version());
    }

    int testTensor()
    {
        const std::vector<std::int64_t> dims = {1, 5, 12};
        const auto data_size = std::accumulate(dims.begin(), dims.end(), sizeof(float), std::multiplies<std::int64_t>{});

        auto data = static_cast<float*>(std::malloc(data_size));

        std::vector<float> vals = {
                -0.4809832f, -0.3770838f, 0.1743573f, 0.7720509f, -0.4064746f, 0.0116595f, 0.0051413f, 0.9135732f, 0.7197526f, -0.0400658f, 0.1180671f, -0.6829428f,
                -0.4810135f, -0.3772099f, 0.1745346f, 0.7719303f, -0.4066443f, 0.0114614f, 0.0051195f, 0.9135003f, 0.7196983f, -0.0400035f, 0.1178188f, -0.6830465f,
                -0.4809143f, -0.3773398f, 0.1746384f, 0.7719052f, -0.4067171f, 0.0111654f, 0.0054433f, 0.9134697f, 0.7192584f, -0.0399981f, 0.1177435f, -0.6835230f,
                -0.4808300f, -0.3774327f, 0.1748246f, 0.7718700f, -0.4070232f, 0.0109549f, 0.0059128f, 0.9133330f, 0.7188759f, -0.0398740f, 0.1181437f, -0.6838635f,
                -0.4807833f, -0.3775733f, 0.1748378f, 0.7718275f, -0.4073670f, 0.0107582f, 0.0062978f, 0.9131795f, 0.7187147f, -0.0394935f, 0.1184392f, -0.6840039f,
        };

        std::copy(vals.begin(), vals.end(), data); // init input_vals.

        auto tensor = TF_NewTensor(TF_FLOAT,
                                   dims.data(), static_cast<int>(dims.size()),
                                   data, data_size,
                                   DeallocateTensor, nullptr);
        SCOPE_EXIT{ TF_DeleteTensor(tensor); }; // Auto-delete on scope exit.

        if (tensor == nullptr) {
            std::cout << "Wrong creat tensor" << std::endl;
            return 1;
        }

        if (TF_TensorType(tensor) != TF_FLOAT) {
            std::cout << "Wrong tensor type" << std::endl;
            return 2;
        }

        if (TF_NumDims(tensor) != static_cast<int>(dims.size())) {
            std::cout << "Wrong number of dimensions" << std::endl;
            return 3;
        }

        for (std::size_t i = 0; i < dims.size(); ++i) {
            if (TF_Dim(tensor, static_cast<int>(i)) != dims[i]) {
                std::cout << "Wrong dimension size for dim: " << i << std::endl;
                return 4;
            }
        }

        if (TF_TensorByteSize(tensor) != data_size) {
            std::cout << "Wrong tensor byte size" << std::endl;
            return 5;
        }

        auto tensor_data = static_cast<float*>(TF_TensorData(tensor));

        if (tensor_data == nullptr) {
            std::cout << "Wrong data tensor" << std::endl;
            return 6;
        }

        for (std::size_t i = 0; i < vals.size(); ++i) {
            if (tensor_data[i] != vals[i]) {
                std::cout << "Element: " << i << " does not match" << std::endl;
                return 7;
            }
        }

        std::cout << "Success created tensor" << std::endl;
        std::cout << static_cast<float*>(TF_TensorData(tensor))[0] << std::endl;

        return 0;
    }
};

#endif //TENSORFLOW_VOXELGRID_VOXELGRID_H
