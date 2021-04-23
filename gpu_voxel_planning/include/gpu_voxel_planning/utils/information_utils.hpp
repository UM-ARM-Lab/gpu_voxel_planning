//
// Created by bsaund on 4/23/21.
//

#ifndef GPU_VOXEL_PLANNING_INFORMATION_UTILS_HPP
#define GPU_VOXEL_PLANNING_INFORMATION_UTILS_HPP

#include <map>
#include <vector>
#include <cmath>
#include <arc_utilities/pretty_print.hpp>

namespace GVP {

double calcEntropy(double num_elements) {
  return std::log(num_elements);
}

double calcEntropy(const std::vector<int>& possible_measurements) {
  return calcEntropy((double)possible_measurements.size());
}

double calcConditionalEntropy(const std::vector<int>& possible_measurements){
  std::map<int, int> measurement_count;

  for(const auto& measurement: possible_measurements){
    if(measurement_count.count(measurement) == 0){
      measurement_count[measurement] = 0;
    }
    measurement_count[measurement] += 1;
  }

//  std::cout << PrettyPrint::PrettyPrint(measurement_count) << "\n";

  double entropy = 0;
  for(const auto& count: measurement_count){
    entropy += count.second * calcEntropy(count.second) / possible_measurements.size();
  }
  return entropy;
}


double calcIG(const std::vector<int>& possible_measurements){
  return calcEntropy(possible_measurements) - calcConditionalEntropy(possible_measurements);
}


}

#endif  // GPU_VOXEL_PLANNING_INFORMATION_UTILS_HPP
