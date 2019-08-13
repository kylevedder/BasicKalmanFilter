#include <iostream>
#include <random>

template <typename T>
constexpr T Sq(const T& v) {
  return v * v;
}

template <typename T>
constexpr T Inv(const T& v) {
  if (v == 0) {
    return 0;
  }
  return (1 / v);
}

template <typename T>
T Sqrt(const T& v) {
  return static_cast<T>(sqrt(v));
}

template <typename T>
class Gaussian {
  std::default_random_engine generator_;
  std::normal_distribution<T> randn_;

 public:
  T mean;
  T std_dev;

  Gaussian() = delete;
  Gaussian(const T mean, const T std_dev)
      : generator_(), randn_(mean, std_dev), mean(mean), std_dev(std_dev) {}

  float GetSample() { return randn_(generator_); }
};

using Gaussianf = Gaussian<float>;

Gaussianf KalmanFilter(const Gaussianf& state, const float control,
                       const float observation) {
  const Gaussianf transition_uncertanty(0, 0.1);
  const Gaussianf measurement_noise(0, 0.1);
  const float A = 1.0f;
  const float B = 1.0f;
  const float C = 1.0f;
  std::cout << "Given mean: " << state.mean << '\n';
  std::cout << "Given std dev: " << state.std_dev << '\n';
  std::cout << "Control: " << control << '\n';
  std::cout << "Observation: " << observation << '\n';

  const float pred_mean = A * state.mean + B * control;
  std::cout << "Pred mean: " << pred_mean << '\n';
  const float pred_std_dev =
      A * state.std_dev * A + transition_uncertanty.std_dev;
  std::cout << "Pred std dev: " << pred_std_dev << '\n';
  const float kalman_gain =
      pred_std_dev * C * Inv(C * pred_std_dev * C + measurement_noise.std_dev);
  std::cout << "Kalman gain: " << kalman_gain << '\n';
  const float mean = pred_mean + kalman_gain * (observation - C * pred_mean);
  const float std_dev = (1 - kalman_gain * C) * pred_std_dev;
  return {mean, std_dev};
}

int main() {
  Gaussianf state(0, 0);
  for (int i = 0; i < 5; ++i) {
    const auto command = 1;
    const auto true_state = i + command;
    const auto observe_state = i + command;
    state = KalmanFilter(state, command, true_state);
    std::cout << "Pred state: (" << state.mean << ", " << state.std_dev
              << ") True: " << true_state << "\n";
    std::cout << "==============\n";
  }
  return 0;
}