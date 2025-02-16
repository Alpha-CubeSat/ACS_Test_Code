#include <ArduinoEigen.h>
#include <vector>

class EKF {
public:
  EKF();

  void initialize(double delta_t, const Eigen::VectorXd &initial_state,
                  const Eigen::MatrixXd &initial_covariance,
                  const Eigen::MatrixXd &process_noise_covariance,
                  const Eigen::MatrixXd &noise_covariance,
                  const Eigen::MatrixXd &Hd);
  void step();

  Eigen::VectorXd state;
  Eigen::VectorXd Z;
  Eigen::MatrixXd covariance;

protected:
  double dt;
  Eigen::MatrixXd Q;   // Process noise covariance
  Eigen::MatrixXd R_d; // (measurement noise variance) Matrices
  Eigen::MatrixXd H_d;
  void predict(const Eigen::MatrixXd &J_k_k);
  void correct();
  Eigen::VectorXd rk4(const Eigen::VectorXd &x_initial, double f_step_size,
                      double t_start, double t_end);
  Eigen::MatrixXd CalculateJacobian();
};