#pragma once

// #include "utils.h"
#include <matrix/math.hpp>

/// Represents a Rigid body
class RigidBody {
private:
public:
  /// Loads the rigid body properties from the yaml file
  // RigidBody(std::string parameter_path);
  // RigidBody(){};

  // Variables
protected:
  // Mass and Inertia Matrix ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Mass of the rigid body
  float mass_{};

  // Inertia matrix of the rigid body

  matrix::SquareMatrix<float, 3> inertia_matrix_;
  // Position and Orietnation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Position of the rigid body
  matrix::Vector3f position_;

  // /// Derivative of position represenation of the rigid body
  // matrix::Vector3f position_dot_;

  /// Velocity of the rigid body
  matrix::Vector3f velocity_;

  /// velocity of rigid body
  matrix::Vector3f acceleration_;

  /// Orientation of the rigid body as Quaternion (q_0, q_x, q_y, q_z)
  matrix::Quatf orientation_;

  /// Time derivative of quaternion representing rigid body orientation
  matrix::Quatf orientation_dot_;

  /// Angular position_dot of the rigid body
  matrix::Vector3f angular_velocity_;

  /// Orientation as Z-Y-X Euler angle
  matrix::Eulerf euler_orientation_;

  /// Angular acceleration of the rigid body
  matrix::Vector3f angular_acceleration_;

  // Getter Functions
public:
  /// Getter function
  const float &mass() const { return mass_; }

  /// Getter function
  const matrix::SquareMatrix<float, 3> &inertia_matrix() const {
    return inertia_matrix_;
  }

  /// Getter function
  const matrix::Vector3f &position() const { return position_; }

  // /// Getter function
  // const matrix::Vector3f &position_dot() const { return position_dot_;
  // }

  /// Getter function
  const matrix::Vector3f &velocity() const { return velocity_; }

  /// Getter function
  const matrix::Vector3f &acceleration() const { return acceleration_; }

  /// Getter function
  const matrix::Quatf &orientation() const { return orientation_; }

  /// Getter function
  const matrix::Eulerf &euler_orientation() const { return euler_orientation_; }

  /// Getter function
  const matrix::Quatf &orientation_dot() const { return orientation_dot_; }

  /// Getter function
  const matrix::Vector3f &angular_velocity() const { return angular_velocity_; }

  /// Getter function
  const matrix::Vector3f &angular_acceleration() const {
    return angular_acceleration_;
  }

  /// Setter function
  void set_mass(float mass) { mass_ = mass; }

  /// Setter function
  void set_inertia_matrix(float data[3][3]) {
    matrix::SquareMatrix<float, 3> inertia_matrix(data);
    inertia_matrix_ = inertia_matrix;
  }

  /// Setter function
  void set_position(matrix::Vector3f position) { position_ = position; }

  // /// Setter function
  // void set_position_dot(matrix::Vector3f position_dot) {
  //   position_dot_ = position_dot;
  // }

  void set_velocity(matrix::Vector3f velocity) { velocity_ = velocity; }

  /// Setter function
  void set_orientation(matrix::Quatf orientation) {
    orientation_ = orientation;
    set_euler_orientation();
  }

  void set_euler_orientation() {
    matrix::Eulerf euler_orientation(orientation_);
    euler_orientation_ = euler_orientation * (180 / M_PI);
  }

  /// Setter function
  void set_angular_velocity(matrix::Vector3f angular_velocity) {
    angular_velocity_ = angular_velocity;
  }

  void set_state(const matrix::Vector3f position,
                 const matrix::Vector3f velocity,
                 const matrix::Quatf orientation,
                 const matrix::Vector3f angular_velocity) {

    position_ = position;
    velocity_ = velocity;
    orientation_ = orientation;
    angular_velocity_ = angular_velocity;

    // Update euler angle from orientation quaternion
    set_euler_orientation();
  }
};
