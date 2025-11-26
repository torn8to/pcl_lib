#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

//**
// A Sliding winodow average of N size to update
// @brief template must be static_castable from an integer and have overloads + and / operation
//
// @param size the size of the sliding window
//*/

template <typename T> class SlidingWindowAverage {
public:
  using value_type = T;

  SlidingWindowAverage(std::size_t size) { max_size_ = size; }

  void resize(std::size_t new_size) { max_size_ = new_size; }

  void update(T &value) {
    if (window_.size() == max_size_) {
      window_.pop_back();
    }
    window_.emplace_front();
  }

  T get_average() {
    T accumulator;
    std::for_each(window_.begin(), window_.end(), [&](const T value) { accumulator = value; });
    return accumulator / static_cast<T>(window_.size());
  }

private:
  std::deque<T> window_;
  std::size_t max_size_;
};