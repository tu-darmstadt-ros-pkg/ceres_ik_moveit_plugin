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

  Transform() {}

  Transform(const QuaternionT<T>& _rotation, const Vector3T<T>& _translation) {
    rotation = _rotation;
    translation = _translation;
  }
  Transform(const Transform<double>& trans_d) {
    rotation.x() = T(trans_d.rotation.x());
    rotation.y() = T(trans_d.rotation.y());
    rotation.z() = T(trans_d.rotation.z());
    rotation.w() = T(trans_d.rotation.w());

    for (unsigned int i = 0; i < 3; i++) {
      translation(0) = T(trans_d.translation(i));
    }
  }

  Transform operator *(const Transform<T>& lhs) const {
    return Transform(lhs.rotation * rotation, lhs.rotation * translation + lhs.translation);
  }

  Transform inverse() {
    return Transform(rotation.inverse(), -1 * rotation.inverse() * translation);
  }
};

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

protected:
  std::string name_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d axis_;

  double upper_limit_;
  double lower_limit_;
};

class FixedJoint : public Joint {
public:
  FixedJoint(std::string name, Eigen::Vector3d origin)
    : Joint(name, origin) {}

  template<typename T> Transform<T> pose(T q) const {
    return Transform<T>();
  }

  bool isActuated() const {
    return false;
  }
};

class RevoluteJoint : public Joint {
public:
  RevoluteJoint(std::string name, Eigen::Vector3d origin, Eigen::Vector3d axis, double upper_limit, double lower_limit)
    : Joint(name, origin, axis, upper_limit, lower_limit) {}

  template<typename T> Transform<T> pose(T q) const {
    Vector3T<T> axis_t = vectorDtoT<T>(axis_);
    Vector3T<T> origin_t = vectorDtoT<T>(origin_);
    return Transform<T>(QuaternionT<T>(Eigen::AngleAxis<T>(q, axis_t)), origin_t);
  }

  bool isActuated() const {
    return true;
  }
};

template <typename T> Transform<T> Joint::pose(T q) const {
  if (const FixedJoint* joint = dynamic_cast<const FixedJoint*>(this)) {
    joint->pose(q);
  }
  else if (const RevoluteJoint* joint = dynamic_cast<const RevoluteJoint*>(this)) {
    joint->pose(q);
  }
  else {
    std::cerr << "Dynamic cast failed!" << std::endl;
  }
}

class Link {
public:
  Link(std::string name, const Transform<double>& tip_transform, boost::shared_ptr<Joint> joint)
    : name_(name), tip_transform_(joint->pose<double>(0).inverse() * tip_transform), joint_(joint) {}

  template<typename T> Transform<T> pose(T q) const {
    return joint_->pose(q) * Transform<T>(tip_transform_);
  }

  boost::shared_ptr<Joint> getJoint() const {
    return joint_;
  }

  std::string getName() const {
    return name_;
  }

private:
  std::string name_;
  Transform<double> tip_transform_;
  boost::shared_ptr<Joint> joint_;
};

geometry_msgs::Pose transformToMsg(const Transform<double>& transform);

Transform<double> msgToTransform(const geometry_msgs::Pose& msg);
}
#endif
