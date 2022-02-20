#include <algorithm>
#include <math.h>
#include <random>

#include "ekf.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace prediction
{
void plot(Eigen::Vector4d truth, Eigen::Vector4d pred, Eigen::Vector4d obs) {
  // plot planned path
  static std::vector<double> truth_x(1), truth_y(1);
    truth_x.push_back(truth(State::X));
    truth_y.push_back(truth(State::Y));
  plt::plot(truth_x, truth_y, "r-");

  static std::vector<double> pred_x(1), pred_y(1);
    pred_x.push_back(pred(State::X));
    pred_y.push_back(pred(State::Y));
  plt::plot(pred_x, pred_y, "k-");

    std::vector<double> obs_x(1), obs_y(1);
    obs_x.push_back(obs(State::X));
    obs_y.push_back(obs(State::Y));
    plt::scatter(obs_x, obs_y, 16.0);

  plt::pause(0.01);

  //plt::show();
  plt::save("../doc/groundtruth_state_predicted_states.png");
}

EKF::EKF() {
  initializeKF();
  estimate();
}

void EKF::initializeKF() {

  // covariance matrix of process noise [x y yaw v]
  Q <<  0.15,     0,      0,     0,
           0,  0.15,      0,     0,
           0,     0,   0.21,     0,
           0,     0,      0,  0.25;

  // prediction covariance matrix
  Pest << Eigen::Matrix4d::Identity() * 0.01;
  //Eigen::MatrixXd Pest = Eigen::Matrix<double, 4, 4>::Identity();

}


void EKF::estimate() {
  double dt = 0.1;
  double end_time = 90;

  int cnt = 0;
  for (double time = .0; time < end_time; time += dt) {
    cnt++;

    //update simulated ground truth and get new control inputs
    simulate(time, dt);

    // prediction step with new control inputs
    predictionUpdate(time, u);

    //std::cout << std::endl << "xTruth:\n" << xTruth << std::endl
    //  << "xEst:\n" << xEst << std::endl;

    if (cnt%1 == 0) {
      // observation vector [x y yaw v]
      Eigen::Vector4d z;
      z = getObservation();  // get contaminated sensor data

      // covariance of observation noise
      Eigen::Matrix4d R;
      R <<  0.515,      0,      0,      0,
                0,  0.515,      0,      0,
                0,      0,   0.51,      0,
                0,      0,      0,   0.55;

      // update step
      observationUpdate(z, R);
      if (cnt%10 == 0) plot(xTruth, xEst, z);
    }

    //std::cout << "Pest:\n" << Pest << std::endl;

    //std::cout << std::endl << "xTruth:\n" << xTruth << std::endl
    //  << "xEst:\n" << xEst << std::endl;
  }
}

void EKF::observationUpdate(Eigen::Vector4d z, Eigen::Matrix4d R) {
  // observation matrix
  Eigen::Matrix4d H = Eigen::Matrix4d::Identity();

  // Kalman gain
  Eigen::MatrixXd K = Pest * H.transpose() * (H * Pest * H.transpose() + R).inverse();

  xEst = xEst + K * (z - H * xEst);
  Pest = (Eigen::Matrix4d::Identity() - K * H) * Pest;
}

void EKF::predictionUpdate(const double& current_time, Eigen::Vector2d u) {
  static double last_time = .0;
  double dt = current_time - last_time;
  last_time = current_time;

  // prediction step: update predicted states
  //Eigen::Vector4d imu_data = getImu();
  motionUpdate(dt, xEst, u);
  //std::cout << "xEst:" << std::endl << xEst << std::endl;

  // jacobian of state function dA/dx
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = - u(IMU::VX_IN) * sin(xEst(State::YAW)) * dt;
  F(0, 3) = cos(xEst(State::YAW)) * dt;
  F(1, 2) = u(IMU::VX_IN) * cos(xEst(State::YAW)) * dt;
  F(1, 3) = sin(xEst(State::YAW)) * dt;

  std::cout << std::endl;
  //std::cout << "dA/dx:\n" << F << std::endl;
  std::cout << "Pest:\n" << Pest << std::endl;
  std::cout << "F * Pest * F.transpose():\n" << F * Pest * F.transpose() << std::endl;
  //std::cout << "Q:\n" << Q << std::endl;
  std::cout << "F * Pest * F.transpose() + Q:\n" << F * Pest * F.transpose() + Q << std::endl;

  //std::cout << "Pest:" << std::endl << Pest << std::endl;
  Pest = F * Pest * F.transpose() + Q;
  //std::cout << "updated Pest:" << std::endl << Pest << std::endl;
}

/**
 * @brief update simulated control inputs and simulated ground truth
 * 
 * @param time 
 */
void EKF::simulate(const double& time, const double& dt) {
  // control input [v yaw_rate]
  u << 1.0 * (1 - exp(-time / 10.0)), 0.05 * (1 - exp(-time / 10.0));
  //std::cout << "\nu:\n" << u << std::endl;

  Eigen::Vector2d uTruth(u);
  // R
  Eigen::Vector2d contCov{0.52232,0.39225};
  std::random_device device_random;
  std::default_random_engine generator(device_random());
  for (int i=0; i < uTruth.size(); i++){
    std::normal_distribution<double> noise(0, contCov(i));
    uTruth(i) += noise(generator);
  }
  //std::cout << "uTruth:\n" << uTruth << std::endl;

  // update ground truth poses
  motionUpdate(dt, xTruth, uTruth);

}

/**
 * @brief update state vecotr by feeding last state, control inputs and current time step
 * 
 * @param current_time time when this function called. Used for calculating time step
 * @param x [x y yaw v] updated states
 * @param cont control inputs
 */
void EKF::motionUpdate(const double& dt, //current_time,
                 Eigen::Vector4d& x, const Eigen::Vector2d& cont) {
  //static double last_time = .0;
  // dicretized time step
  //double dt = current_time - last_time;

  // dynamics
  Eigen::Matrix4d A;
  A << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 0;
  Eigen::Matrix<double, 4, 2> B;
  B << dt * cos(x(State::YAW)),   0,
       dt * sin(x(State::YAW)),   0,
                    0,  dt,
                    1,   0;
  //std::cout << "B\n" << B << std::endl;

  x = A * x + B * cont;
  //std::cout << x(State::YAW) << std::endl;

  //last_time = current_time;
}

Eigen::Vector4d EKF::getObservation() {
  /* Simulation parameters */
  // R
  Eigen::Vector4d obsCov{0.515, 0.515, 0.527, 0.225};

  Eigen::Vector4d obs{xTruth};
  // noise
  std::random_device device_random;
  std::default_random_engine generator(device_random());
  for (int i=0; i < obs.size(); i++){
    std::normal_distribution<double> noise(0, obsCov(i));
    obs(i) += noise(generator);
  }

  //std::cout << "obs truth: " << std::endl << xTruth << std::endl
  //  << "obs contaminated: " << std::endl << obs << std::endl;

  return obs;
}




}  // namespace prediction


int main() {
  prediction::EKF kf;




  return 0;
}