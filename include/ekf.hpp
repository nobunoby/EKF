#pragma once

#include <complex>
#include <functional>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

namespace prediction
{
enum State {X, Y, YAW, VX, STATES};
enum Obs {X_OBS, Y_OBS, YAW_OBS, VX_OBS, OBSS};
enum IMU {VX_IN, YAWRATE_IN, INPUTS};

class EKF {
 public:
  EKF();
  void initializeKF();
  void initializeSim();
  void estimate();
  void motionUpdate(const double& dt, //current_time,
                 Eigen::Vector4d& x, const Eigen::Vector2d& cont);
  void predictionUpdate(const double& time, Eigen::Vector2d u);
  void observationUpdate(Eigen::Vector4d z, Eigen::Matrix4d R);
  Eigen::Vector4d getObservation();
  Eigen::Vector4d getImu();

  // simulation methods
  void simulate(const double& time, const double& dt);


 private:
  /*  Kalman Filter parameters */
  // ground true states [x y yaw v]
  Eigen::Vector4d xTruth{0, 0, 0, 0};

  // estimated states by KF
  Eigen::Vector4d xEst{xTruth};

  // covariance matrix for process noise of KF
  Eigen::Matrix<double, 4, 4> Q;

  // intial covariance of estimation of KF
  Eigen::Matrix<double, 4, 4> Pest;

  // cov matrix for observation noise of KF
  //Eigen::Vector4d vR{2.25, 2.25, 0.0027, 0.0025};
  //Eigen::Matrix<double, 4, 4> R = vR.array().matrix().asDiagonal();



  Eigen::Vector2d u;

};

}  // namespace EKF
