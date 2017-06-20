//=================================================================================================
// Copyright (c) 2017, Martin Oehler, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef CARTESIAN_ERROR_H
#define CARTESIAN_ERROR_H

#include <ceres/ceres.h>

#include <ceres_ik_moveit_plugin/transforms.h>

namespace ceres_ik_moveit_plugin {

struct CartesianError {
  CartesianError(const std::vector<Link>& chain, const Transform<double>& target_pose)
    : chain_(chain), target_pose_(target_pose) {}

  template<typename T>
  bool operator()(const T* const joint_angles, T* residuals) const {
    int current_joint_idx = 0;

    Transform<T> pose;
    for (unsigned int i = 0; i < chain_.size(); i++) {
      T q;
      if (chain_[i].getJoint()->isActuated()) {
        q = joint_angles[current_joint_idx];
        current_joint_idx++;
      } else {
        q = T(0.0);
      }
      pose = pose * chain_[i].pose<T>(q);
    }

    Transform<T> target_pose_t(target_pose_);
    Vector3T<T> translation_diff = target_pose_t.translation - pose.translation;

    QuaternionT<T> rotation_diff = QuaternionT<T>(pose.rotation.toRotationMatrix().transpose() * target_pose_t.rotation);
    Vector3T<T> ypr = rotation_diff.toRotationMatrix().eulerAngles(2, 1, 0);

    for (unsigned int i = 0; i < 3; i++) {
      residuals[i] = translation_diff(i);
      residuals[i+3] = ypr[i];
    }



    return true;
  }

  static ceres::CostFunction* Create(const std::vector<Link>& chain, const Transform<double>& target_pose, int num_actuated_joints)
  {
    ceres::CostFunction * cost_function =
        new ceres::AutoDiffCostFunction<CartesianError, ceres::DYNAMIC, 6>(
          new CartesianError(chain, target_pose), num_actuated_joints);
    return cost_function;
  }

  std::vector<Link> chain_;
  Transform<double> target_pose_;
};

}

#endif
