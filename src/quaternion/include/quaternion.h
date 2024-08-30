#ifndef _QUATERNION_H
#define _QUATERNION_H

#include <array>
#include <cmath>
#include <string>

namespace quaternion {
class Quaternion {
 public:
  Quaternion();  //无参构造

  Quaternion(float w, float x, float y, float z);  //有参构造

  Quaternion(const Quaternion &q);  //拷贝构造

  ~Quaternion();  //析构函数

  /*
   * @brief normalize a quaternion
   * @param None
   * @return normalized quaternion
   * @note Noneros
   */
  inline void normalization() {
    float norm = sqrt(std::pow(w_, 2) + std::pow(x_, 2) + std::pow(y_, 2) +
                      std::pow(z_, 2));
    this->w_ /= norm;
    this->x_ /= norm;
    this->y_ /= norm;
    this->z_ /= norm;
  }

  /*
   * @brief set a new quaternion
   * @param w
   * @param x
   * @param y
   * @param z
   * @return None
   * @note None
   */
  inline void setQuat(const float &w, const float &x, const float &y,
                      const float &z) {
    this->w_ = w;
    this->x_ = x;
    this->y_ = y;
    this->z_ = z;
    normalization();
  }

  /*
   * @brief return w_
   * @param None
   * @return w_
   * @note None
   */
  inline float getW() const { return this->w_; }

  /*
   * @brief return x_
   * @param None
   * @return x_
   * @note None
   */
  inline float getX() const { return this->x_; }

  /*
   * @brief return y_
   * @param None
   * @return y_
   * @note None
   */
  inline float getY() const { return this->y_; }

  /*
   * @brief return z_
   * @param None
   * @return z_
   * @note None
   */
  inline float getZ() const { return this->z_; }

  std::array<std::array<float, 3>, 3> toRotationMatrix()
      const;  //四元数转旋转矩阵

  std::array<float, 3> toEuler(
      const std::string &seq) const;  //四元数转euler角---指定某个序列：ZYX，ZYZ

  std::array<float, 3> toEulerZYX() const;  //四元数转ZYX
  std::array<float, 3> toEulerZYZ() const;  //四元数转ZYZ

 private:
  float w_, x_, y_, z_;
};

}  // namespace quaternion

#endif