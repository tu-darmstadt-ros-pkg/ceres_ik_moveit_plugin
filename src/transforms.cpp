#include <ceres_ik_moveit_plugin/transforms.h>

namespace ceres_ik_moveit_plugin {

geometry_msgs::Pose transformToMsg(const Transform<double>& transform) {
  geometry_msgs::Pose msg;
  msg.position.x = transform.translation(0);
  msg.position.y = transform.translation(1);
  msg.position.z = transform.translation(2);

  msg.orientation.x = transform.rotation.x();
  msg.orientation.y = transform.rotation.y();
  msg.orientation.z = transform.rotation.z();
  msg.orientation.w = transform.rotation.w();

  return msg;
}

Transform<double> msgToTransform(const geometry_msgs::Pose& msg) {
  return Transform<double>(Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
                   Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z));
}

}
