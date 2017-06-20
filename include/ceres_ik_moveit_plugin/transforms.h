#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include <iostream>

#include <geometry_msgs/Pose.h>

namespace ceres_ik_moveit_plugin {

template<typename T> using QuaternionT = Eigen::Quaternion<T>;
template<typename T> using Vector3T = Eigen::Matrix<T, 3, 1>;

template<typename T> Vector3T<T> vectorDtoT(const Eigen::Vector3d& vec) {
  return Vector3T<T>(T(vec(0)), T(vec(1)), T(vec(2)));
}

template<typename T> struct Transform {
  QuaternionT<T> rotation;
  Vector3T<T> translation;

  Transform()
    : rotation(QuaternionT<T>(T(1.0), T(0.0), T(0.0), T(0.0))), translation(Vector3T<T>(T(0.0), T(0.0), T(0.0))) {}

  Transform(const QuaternionT<T>& _rotation, const Vector3T<T>& _translation) {
    rotation = _rotation;
    translation = _translation;
  }

  Transform inverse() {
    return Transform(rotation.inverse(), -1 * rotation.inverse() * translation);
  }

  std::string toString() {
    std::stringstream ss;
    ss << "[" << translation(0) << ", " << translation(1) << ", " << translation(2) << "; "
       << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << "]";
    return ss.str();
  }
};

template <typename T> Transform<T> operator *(const Transform<T>& lhs, const Transform<T>& rhs);

class Joint {
public:
  Joint(std::string name, Eigen::Vector3d origin, Eigen::Vector3d axis, double upper_limit, double lower_limit) :
    name_(name), origin_(origin), axis_(axis), upper_limit_(upper_limit), lower_limit_(lower_limit) {}

  Joint(std::string name, Eigen::Vector3d origin) :
    name_(name), origin_(origin), axis_(Eigen::Vector3d::Zero()), upper_limit_(0), lower_limit_(0) {}

  template<typename T> Transform<T> pose(T q) const;

  virtual bool isActuated() const = 0;

  double getUpperLimit() const {
    return upper_limit_;
  }

  double getLowerLimit() const {
    return lower_limit_;
  }

  std::string getName() const {
    return name_;
  }

  Eigen::Vector3d getOrigin() const {
    return origin_;
  }

  Eigen::Vector3d getAxis() const {
    return axis_;
  }

protected:
  std::string name_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d axis_;

  double upper_limit_;
  double lower_limit_;
};

class FixedJoint : public Joint {
public:
  FixedJoint(std::string name, Eigen::Vector3d origin);
  template<typename T> Transform<T> pose(T q) const;
  bool isActuated() const;
};

class RevoluteJoint : public Joint {
public:
  RevoluteJoint(std::string name, Eigen::Vector3d origin, Eigen::Vector3d axis, double upper_limit, double lower_limit);
  Transform<double> pose(double q) const;
  template<typename T> Transform<T> pose(T q) const;
  bool isActuated() const;
};

class Link {
public:
  Link(std::string name, const Transform<double>& tip_transform, boost::shared_ptr<Joint> joint);
  //Transform<double> pose(double q) const;
  template<typename T> Transform<T> pose(T q) const;
  boost::shared_ptr<Joint> getJoint() const;
  std::string getName() const;
  Transform<double> getTipTransform() const;

private:
  std::string name_;
  Transform<double> tip_transform_;
  boost::shared_ptr<Joint> joint_;
};

geometry_msgs::Pose transformToMsg(const Transform<double>& transform);
Transform<double> msgToTransform(const geometry_msgs::Pose& msg);

template <typename T> void convertTransform(const Transform<double>& t1, Transform<T>& t2);
template <> void convertTransform(const Transform<double>& t1, Transform<double>& t2);

}
#endif
