#include <iostream>
#include <random>

std::random_device random_device;
static thread_local std::mt19937_64 random_engine(random_device());

int main(int /*argc*/, char * /*args*/[]) {

  std::cout.precision(std::numeric_limits<double>::max_digits10);
  constexpr unsigned long iterations = 1L << 30;
  std::cout << "Iteration count: " << iterations << std::endl;

  std::uniform_real_distribution<double> r_coeff(0, 1);

  double naive_sum_d = 0;
  double div_first_avg_d = 0;
  float naive_sum_f = 0;
  float div_first_avg_f = 0;

  float kahan_sum_f = 0;
  float kahan_sum_error_f = 0.0;

  float welford_avg_f = 0.0;

  for (unsigned long i = 0; i < iterations; i++) {
    double coeff = r_coeff(random_engine);
    naive_sum_d += coeff;
    div_first_avg_d += coeff / (double)iterations;
    naive_sum_f += (float)coeff;
    div_first_avg_f += (float)coeff / (float)iterations;

    float y = (float)coeff - kahan_sum_error_f;
    float t = kahan_sum_f + y;
    kahan_sum_error_f = (t - kahan_sum_f) - y;
    kahan_sum_f = t;

    auto n = i + 1;
    welford_avg_f += ((float)coeff - welford_avg_f) / (float)n;
  }

  std::cout << "Naive sum (double): " << naive_sum_d << std::endl;
  std::cout << "Naive sum (float):  " << naive_sum_f << std::endl;
  std::cout << "Kahan sum (float):  " << kahan_sum_f << std::endl;

  double naive_avg_d = naive_sum_d / (double)iterations;
  float naive_avg_f = naive_sum_f / (float)iterations;
  float kahan_avg_f = kahan_sum_f / (float)iterations;

  std::cout << std::endl;
  std::cout << "Naive average (double):     " << naive_avg_d << std::endl;
  std::cout << "Div first average (double): " << div_first_avg_d << std::endl;

  std::cout.precision(std::numeric_limits<float>::max_digits10);
  std::cout << "Naive average (float):      " << naive_avg_f << std::endl;
  std::cout << "Div first average (float):  " << div_first_avg_f << std::endl;
  std::cout << "Kahan average (float):      " << kahan_avg_f << std::endl;
  std::cout << "Welford average (float):    " << welford_avg_f << std::endl;
}
