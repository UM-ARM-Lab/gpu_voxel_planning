//
// Created by bsaund on 12/10/20.
//
#include <vector>
//#include <jsoncpp/json/json.h>
#include <hjson/hjson.h>

#ifndef GPU_VOXEL_PLANNING_JSON_HELPERS_H
#define GPU_VOXEL_PLANNING_JSON_HELPERS_H

namespace Json {

    template<typename T>
    std::vector<T> toVector(const Hjson::Value &val);

    template<>
    std::vector<double> toVector(const Hjson::Value &val) {
        std::vector<double> v;
        for (int i = 0; i < val.size(); i++) {
            v.push_back(val[i]);
        }
        return v;
    }

    template<>
    std::vector<float> toVector(const Hjson::Value &val) {
        std::vector<float> v;
        for (int i = 0; i < val.size(); i++) {
            v.push_back(val[i]);
        }
        return v;
    }
}

#endif  // GPU_VOXEL_PLANNING_JSON_HELPERS_H
