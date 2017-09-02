#ifndef MPC_H
#define MPC_H
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class MPC {
 public:
  MPC();
  virtual ~MPC();
  // Solve the model given an initial state.
  // Return the next state and actuations as a
  // vector.
  vector<double> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);
  double polyeval(Eigen::VectorXd coeffs, double x);
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
};

#endif /* MPC_H */
