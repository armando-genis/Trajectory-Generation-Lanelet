#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace trajectory_frenet {

class Polynomial {
public:
  Polynomial(const Eigen::Matrix<double,6,1>& boundaries, double T); // Constructor for a 6th degree polynomial (s0, v0, a0, sT, vT, aT)
  Polynomial(const Eigen::Matrix<double,5,1>& boundaries, double T); // Constructor for a 5th degree polynomial (s0, v0, a0, vT, aT)

  ~Polynomial() = default;

  double evaluate_position(double t) const;

  double evaluate_velocity(double t) const;

  double evaluate_acceleration(double t) const;

  double evaluate_jerk(double t) const;

private:
  double T_;                      ///< total time duration
  Eigen::VectorXd pos_coeffs_;    ///< polynomial coefficients for position
  Eigen::VectorXd vel_coeffs_;    ///< polynomial coefficients for velocity
  Eigen::VectorXd acc_coeffs_;    ///< polynomical coefficients for acceleration
  Eigen::VectorXd jerk_coeffs_;   ///< polynomial coefficients for jerk

  static double evaluate(const Eigen::VectorXd& coeffs, double t) noexcept;
  static Eigen::VectorXd derivative(const Eigen::VectorXd& coeffs) noexcept;
};

} // namespace trajectory_frenet