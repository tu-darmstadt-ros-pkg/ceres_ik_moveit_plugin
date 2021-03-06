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

#ifndef POSE_ERROR_H
#define POSE_ERROR_H

#include <ceres/ceres.h>

#include <ceres_ik_moveit_plugin/transforms.h>

namespace ceres_ik_moveit_plugin {

struct PoseError {
  PoseError(const std::vector<Link>& chain, const Transform<double>& ik_pose, double orientation_weight = 0.5)
    : chain_(chain), ik_pose_(ik_pose), orientation_weight_(orientation_weight) {}

  template<typename T>
  bool operator()(T const* const* joint_angles, T* residuals) const {
    int current_joint_idx = 0;

    Transform<T> current_pose;
    for (unsigned int i = 0; i < chain_.size(); i++) {
      T q;
      if (chain_[i].getJoint()->isActuated()) {
        q = joint_angles[0][current_joint_idx];
        current_joint_idx++;
      } else {
        q = T(0.0);
      }
      current_pose = current_pose * chain_[i].pose<T>(q);
    }

    // Translation
    Transform<T> ik_pose_t;
    convertTransform(ik_pose_, ik_pose_t);
    Vector3T<T> translation_diff = ik_pose_t.translation - current_pose.translation;

    // Orientation
    Vector3T<T> p1(T(orientation_weight_), T(0.0), T(0.0));
    Vector3T<T> p2(T(0.0), T(orientation_weight_), T(0.0));

    Vector3T<T> p1_current = current_pose.rotation * p1;
    Vector3T<T> p1_target = ik_pose_t.rotation * p1;

    Vector3T<T> p2_current = current_pose.rotation * p2;
    Vector3T<T> p2_target = ik_pose_t.rotation * p2;

    Vector3T<T> p1_diff = p1_current - p1_target;
    Vector3T<T> p2_diff = p2_current - p2_target;

    for (unsigned int i = 0; i < 3; i++) {
      residuals[i] = translation_diff(i);
      residuals[i+3] = p1_diff(i);
      residuals[i+6] = p2_diff(i);
    }

    return true;
  }

  static ceres::CostFunction* Create(const std::vector<Link>& chain, const Transform<double>& target_pose, int num_actuated_joints, double orientation_weight = 0.5)
  {
    ceres::DynamicAutoDiffCostFunction<PoseError> * cost_function =
        new ceres::DynamicAutoDiffCostFunction<PoseError>(
          new PoseError(chain, target_pose, orientation_weight));

    cost_function->AddParameterBlock(num_actuated_joints);
    cost_function->SetNumResiduals(9);

    return static_cast<ceres::CostFunction*> (cost_function);
  }

  std::vector<Link> chain_;
  Transform<double> ik_pose_;
  double orientation_weight_;
};

}

#endif
