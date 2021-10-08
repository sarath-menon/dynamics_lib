#pragma once
#include "rigidbody.h"

/// Represents the quadcopter
class QuadcopterFrame : public RigidBody {

public:
  /// Loads the quadcopter properties from the yaml file
  QuadcopterFrame();

  // Variables
protected:
  // Geometrical Properties ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Linear drag coefficient
  matrix::Matrix3f linear_drag_coeff_;

  // /// Angular drag coefficient
  matrix::Matrix3f angular_drag_coeff_;

  /// Distance from the quadcopter's center of mass to the propellor
  float arm_length_{};

  // Variables for dynamics function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Rotation matrix - body to inertial frame
  matrix::Dcmf _R_OB;

  /// No need to computer inverse of inertia matrix in each function call
  matrix::SquareMatrix<float, 3> inertia_matrix_inverse_;

  // Constants
private:
  // Acceleration due to gravity
  const matrix::Vector3f gravity_acc = matrix::Vector3f(0, 0, 9.81);

public:
  /// Quadcopter Dynamics
  void dynamics(const matrix::Vector3f body_thrust,
                const matrix::Vector3f body_torque);

  void euler_step(const float dt);

  // Rotation only simulation for tuning attitude controller
  void attitude_tune_euler_step(const float dt);

public:
  /// Getter function
  const float linear_drag_coeff_x() const { return linear_drag_coeff_(0, 0); }
  const float linear_drag_coeff_y() const { return linear_drag_coeff_(1, 1); }
  const float linear_drag_coeff_z() const { return linear_drag_coeff_(2, 2); }

  /// Getter function
  const float angular_drag_coeff_x() const { return angular_drag_coeff_(0, 0); }
  const float angular_drag_coeff_y() const { return angular_drag_coeff_(0, 0); }
  const float angular_drag_coeff_z() const { return angular_drag_coeff_(0, 0); }

  /// Getter function
  const float arm_length() const { return arm_length_; }

public:
  /// Setter function
  void set_linear_drag_coeff(const float data[3]) {
    matrix::Vector3f linear_drag_coeff(data);
    linear_drag_coeff_ = matrix::diag(linear_drag_coeff);
  }

  /// Setter function
  void set_angular_drag_coeff(const float data[3]) {
    matrix::Vector3f angular_drag_coeff(data);
    angular_drag_coeff_ = matrix::diag(angular_drag_coeff);
  }

  /// Setter function
  void set_arm_length(const float arm_length) { arm_length_ = arm_length; }

  /// Setter function
  void set_inertia_matrix_inverse() {
    inertia_matrix_inverse_(0, 0) = 1 / inertia_matrix_(0, 0);
    inertia_matrix_inverse_(1, 1) = 1 / inertia_matrix_(1, 1);
    inertia_matrix_inverse_(1, 1) = 1 / inertia_matrix_(1, 1);
  }
};