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

#ifndef JOINT_ANGLE_REGULARIZATION_H
#define JOINT_ANGLE_REGULARIZATION_H

#include <ceres/ceres.h>

#include <ceres_ik_moveit_plugin/transforms.h>

namespace ceres_ik_moveit_plugin {

struct JointAngleRegularization {
  JointAngleRegularization(const std::vector<double>& start_state, std::vector<double> weights)
    : start_state_(start_state), weights_(weights) {}

  template<typename T>
  bool operator()(T const* const* joint_angles, T* residuals) const {
    for (unsigned int i = 0; i < start_state_.size(); i++) {
      residuals[i] = T(weights_[i]) * (joint_angles[0][i] - T(start_state_[i]));
    }

    return true;
  }

  static ceres::CostFunction* Create(const std::vector<double>& start_state, std::vector<double> weights)
  {
    auto * cost_function =
        new ceres::DynamicAutoDiffCostFunction<JointAngleRegularization>(
          new JointAngleRegularization(start_state, weights));

    cost_function->AddParameterBlock(start_state.size());
    cost_function->SetNumResiduals(start_state.size());

    return static_cast<ceres::CostFunction*> (cost_function);
  }

  std::vector<double> start_state_;
  std::vector<double> weights_;
};

}

#endif
