//
// Created by bsaund on 12/10/20.
//
#include <vector>
#include <jsoncpp/json/json.h>

#ifndef GPU_VOXEL_PLANNING_JSON_HELPERS_H
#define GPU_VOXEL_PLANNING_JSON_HELPERS_H

namespace Json{

template<typename T>
std::vector<T> toVector(const Value& val);

template<>
std::vector<double> toVector(const Value& val){
  std::vector<double> v;
  for(const auto& num: val){
    v.push_back(num.asDouble());
  }
  return v;
}

template<>
std::vector<float> toVector(const Value& val){
  std::vector<float> v;
  for(const auto& num: val){
    v.push_back(num.asFloat());
  }
  return v;
}

}

#endif  // GPU_VOXEL_PLANNING_JSON_HELPERS_H
