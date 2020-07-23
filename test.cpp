#include <iostream>
#include <random>

std::random_device random_device;
static thread_local std::mt19937_64 random_engine(random_device());

int main(int /*argc*/, char * /*args*/[]) {

  std::cout.precision(std::numeric_limits<double>::max_digits10);
  constexpr unsigned long iterations = 1L << 30;
  std::cout << "Iteration count: " << iterations << std::endl;

  std::uniform_real_distribution<double> r_coeff(0, 1);

  double sum = 0;
  double sumdiv = 0;
  float fsum = 0;
  float fsumdiv = 0;

  float sumk = 0;
  float c = 0.0;

  for (unsigned long i = 0; i < iterations; i++) {
    double coeff = r_coeff(random_engine);
    sum += coeff;
    sumdiv += coeff / (double)iterations;
    fsum += (float)coeff;
    fsumdiv += (float)coeff / (float)iterations;

    float y = (float)coeff - c;
    float t = sumk + y;
    c = (t - sumk) - y;
    sumk = t;
  }

  std::cout << "Double result:      " << sum << std::endl;
  std::cout << "Float result:       " << fsum << std::endl;
  std::cout << "Float kahan result: " << sumk << std::endl;

  sum /= (double)iterations;
  fsum /= (float)iterations;
  sumk /= (float)iterations;

  std::cout << std::endl;
  std::cout << "Double result:      " << sum << std::endl;
  std::cout << "Double div result:  " << sum << std::endl;

  std::cout.precision(std::numeric_limits<float>::max_digits10);
  std::cout << "Float result:       " << fsum << std::endl;
  std::cout << "Float div result:   " << fsumdiv << std::endl;
  std::cout << "Float kahan result: " << sumk << std::endl;
}
