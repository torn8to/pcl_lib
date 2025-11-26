#include <chrono>

class ManualTimer {
public:
  void start() { start_ = std::chrono::steady_clock::now(); }

  long long stop() {
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();
  }

private:
  std::chrono::steady_clock::time_point start_;
};
