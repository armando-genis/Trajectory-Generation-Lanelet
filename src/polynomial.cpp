#include "trajectory_frenet/polynomial.hpp"

#include <cassert>
#include <cmath>

namespace trajectory_frenet {
Polynomial::Polynomial(const Eigen::Matrix<double,6,1>& boundaries, double T)
  : T_{T} {
  
  assert(boundaries.size() == 6 && "Polynomial: need 6 boundary values {s0,v0,a0,sT,vT,aT}");
  assert(T > 0 && "Polynomial: time horizon T must be positive");

  // unpack boundary values
  const double s0 = boundaries[0];
  const double v0 = boundaries[1];
  const double a0 = boundaries[2];
  const double sT = boundaries[3];
  const double vT = boundaries[4];
  const double aT = boundaries[5];

  // build the 6x6 matrix for the polynomial coefficients A.c = b
  Eigen::Matrix3d A;
	Eigen::Vector3d B;

	A << 
    std::pow(T_,3),   std::pow(T_,4),    std::pow(T_,5),
		3*std::pow(T_,2), 4*std::pow(T_,3),  5*std::pow(T_,4),
		6*T_,             12*std::pow(T_,2), 20*std::pow(T_,3);

	B << 
    sT - s0 - v0*T_ - 0.5*a0*std::pow(T_,2),
		vT - v0 - a0*T_,
		aT - a0;

	//  Solve for x in Ax=B
	Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);

  // precomputing derivatives
  pos_coeffs_.resize(6);
  pos_coeffs_ << s0, v0, a0/2.0, coefficients[0], coefficients[1], coefficients[2];
  vel_coeffs_ = derivative(pos_coeffs_);
  acc_coeffs_ = derivative(vel_coeffs_);
  jerk_coeffs_ = derivative(acc_coeffs_);
}

Polynomial::Polynomial(const Eigen::Matrix<double,5,1>& boundaries, double T)
 : T_ {T} {
  
  assert(boundaries.size() == 5 && "Polynomial: need 5 boundary values {s0,v0,a0,vT,aT}");
  assert(T > 0 && "Polynomial: time horizon T must be positive");

  // unpack boundary values
  const double s0 = boundaries[0];
  const double v0 = boundaries[1];
  const double a0 = boundaries[2];
  const double vT = boundaries[3];
  const double aT = boundaries[4];

  // build the 5x5 matrix for the polynomial coefficients A.c = b
  Eigen::Matrix2d A;
  Eigen::Vector2d B;

  A << 
    3*pow(T_,2), 4*pow(T_,3),
	  6*T_,        12*pow(T_,2);

  B << 
    vT - v0 - a0*T_,
	  aT - a0;

  Eigen::Vector2d coeffs = A.colPivHouseholderQr().solve(B);

	pos_coeffs_.resize(5);
	pos_coeffs_ << s0, v0, a0/2, coeffs[0], coeffs[1];

	vel_coeffs_ = derivative(pos_coeffs_);
	acc_coeffs_ = derivative(vel_coeffs_);
	jerk_coeffs_ = derivative(acc_coeffs_);
 }

// Generic Horner‐style evaluator
double Polynomial::evaluate(const Eigen::VectorXd& coeffs, double t) noexcept {  
  double result = 0.0;
  double power = 1.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * power;
    power *= t;
  }
  return result;
}

// Compute the derivative coefficient vector
Eigen::VectorXd Polynomial::derivative(const Eigen::VectorXd& coeffs) noexcept {
    int n = coeffs.size();
    if (n < 2) return Eigen::VectorXd();
    Eigen::VectorXd d(n - 1);
    for (int i = 1; i < n; ++i) {
        d[i - 1] = coeffs[i] * i;
    }
    return d;
}

double Polynomial::evaluate_position(double t) const {
  return evaluate(pos_coeffs_, t);
}

double Polynomial::evaluate_velocity(double t) const {
  return evaluate(vel_coeffs_, t);
}

double Polynomial::evaluate_acceleration(double t) const {
  return evaluate(acc_coeffs_, t);
}

double Polynomial::evaluate_jerk(double t) const {
  return evaluate(jerk_coeffs_, t);
}

} // namespace trajectory_frenet