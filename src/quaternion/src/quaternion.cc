#include <array>
#include <cmath>
#include <iostream>

#include "quaternion.h"

namespace quaternion {
Quaternion::Quaternion() : w_(1.0), x_(0.0), y_(0.0), z_(0.0) {}

Quaternion::Quaternion(float w, float x, float y, float z)
    : w_(w), x_(x), y_(y), z_(z) {
  normalization();
}

Quaternion::Quaternion(const Quaternion &q) {
  w_ = q.w_;
  x_ = q.x_;
  y_ = q.y_;
  z_ = q.z_;
  normalization();
}

Quaternion::~Quaternion() {}

/*
 * @brief Converts a quaternion to a 3x3 rotation matrix.
 * @param this The constant reference to the quaternion object to be
 * converted.
 * @return 3x3 array representing the rotation matrix.
 * @note normalization is required
 * @warning without normalization will result in precision loss
 */
std::array<std::array<float, 3>, 3> Quaternion::toRotationMatrix() const {
  std::array<std::array<float, 3>, 3> rotationMatrix;
  rotationMatrix[0][0] = 1 - 2 * y_ * y_ - 2 * z_ * z_;
  rotationMatrix[0][1] = 2 * x_ * y_ - 2 * w_ * z_;
  rotationMatrix[0][2] = 2 * w_ * y_ + 2 * x_ * z_;
  rotationMatrix[1][0] = 2 * x_ * y_ + 2 * w_ * z_;
  rotationMatrix[1][1] = 1 - 2 * x_ * x_ - 2 * z_ * z_;
  rotationMatrix[1][2] = 2 * y_ * z_ - 2 * w_ * x_;
  rotationMatrix[2][0] = 2 * x_ * z_ - 2 * w_ * y_;
  rotationMatrix[2][1] = 2 * w_ * x_ + 2 * y_ * z_;
  rotationMatrix[2][2] = 1 - 2 * x_ * x_ - 2 * y_ * y_;
  return rotationMatrix;
}

/*
 * @brief Converts a quaternion to an euler sequence.
 * @param seq A string specifying the order of rotations for the Euler sequence
 *            (e.g. "ZYX", "ZYZ").
 * @return 1x3 array representing the euler angles.
 * @note
 */
std::array<float, 3> Quaternion::toEuler(const std::string &seq) const {
  std::array<float, 3> euler;
  if (seq == "ZYX") {
    euler = toEulerZYX();
  } else if (seq == "ZYZ") {
    euler = toEulerZYZ();
  } else {
    throw std::invalid_argument("Invalid rotation sequence!");
  }
  return euler;
}

/*
 * @brief Converts a quaternion to ZYX euler angles
 * @param None
 * @return 1x3 array representing ZYX euler angles.
 * @note
 */
std::array<float, 3> Quaternion::toEulerZYX() const {
  std::array<float, 3> eulerZYX;
  auto R = toRotationMatrix();
  eulerZYX[0] = std::atan2(R[1][0], R[0][0]);
  eulerZYX[1] = std::asin(-R[2][0]);
  eulerZYX[2] = std::atan2(R[2][1], R[2][2]);
  return eulerZYX;
}

/*
 * @brief Converts a quaternion to ZYZ euler angles
 * @param None
 * @return 1x3 array representing ZYZ euler angles.
 * @note
 */
std::array<float, 3> Quaternion::toEulerZYZ() const {
  std::array<float, 3> eulerZYZ;
  auto R = toRotationMatrix();
  //   eulerZYZ[1] = -std::acos(R[2][2]);
  eulerZYZ[1] = std::atan2(
      -std::sqrt(std::pow(R[0][2], 2) + std::pow(R[1][2], 2)), R[2][2]);
  eulerZYZ[0] = std::atan2(R[1][2] / std::sin(eulerZYZ[1]),
                           R[0][2] / std::sin(eulerZYZ[1]));
  eulerZYZ[2] = std::atan2(R[2][1] / std::sin(eulerZYZ[1]),
                           -R[2][0] / std::sin(eulerZYZ[1]));
  return eulerZYZ;
}

}  // namespace quaternion
