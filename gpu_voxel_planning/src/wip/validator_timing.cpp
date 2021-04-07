#include <chrono>

#include "victor_planning.hpp"
#include "victor_validator.hpp"

using namespace gpu_voxels_planner;

int main(int argc, char **argv) {
  std::shared_ptr<VictorPlanner> vpln = std::make_shared<VictorPlanner>();
  vpln->vv_ptr->testObstacle();
  auto start = std::chrono::steady_clock::now();

  for (int i = 0; i < 1000; i++) {
    vpln->vv_ptr->isCurrentlyValid();
  }

  auto end = std::chrono::steady_clock::now();

  std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms\n";

  // size_t N = 100000000000; //100 billion
  size_t N = 1000000000;  // 1 billion
  std::vector<int> testbool(N);

  start = std::chrono::steady_clock::now();
  for (size_t ind = 0; ind < N; ind++) {
    testbool[ind] = ind;
  }

  end = std::chrono::steady_clock::now();
  std::cout << "Raw cpu access time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms\n";
}
