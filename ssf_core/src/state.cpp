/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ssf_core/state.h>

namespace ssf_core
{
///状态初始化，25个状态，除了p_wv_, 论文中认为为0
void State::reset(){
  // states varying during propagation
  p_.setZero();
  v_.setZero();
  q_.setIdentity();
  b_w_.setZero();
  b_a_.setZero();

  L_ = 1.0;
  q_wv_.setIdentity();
  q_ci_.setIdentity();
  p_ci_.setZero();

  w_m_.setZero();
  a_m_.setZero();

  q_int_.setIdentity();

  P_.setZero();
  time_ = 0;
}

/**
 * \cov
         p_x_ p_y_ p_z_ theta_x_ theta_y_ theta_z_
p_x_     0    1    2    3        4    	  5
p_y_     6    7    8    9        10       11
p_z_     12   13   14   15       16       17
theta_x_ 18   19   20   21       22       23
theta_y_ 24   25   26   27       28       29
theta_z_ 30   31   32   33       34       35
**/
///将误差状态协方差阵P_中与p_,q_相关的协方差提取出来单独设为cov
void State::getPoseCovariance(geometry_msgs::PoseWithCovariance::_covariance_type & cov)
{
  assert(cov.size() == 36);
  /**
     *\return <cov>{0 1 2 6 7 8 12 13 14}
     * \param <P_> {0 1 2 25 26 27 50 51 52}
     */
  for (int i = 0; i < 9; i++)
    cov[i / 3 * 6 + i % 3] = P_(i / 3 * N_STATE + i % 3);
  /**
     *\return <cov>{3 4 5 9 10 11 15 16 17}
     * \param <P_> {6 7 8 31 31 33 56 57 58}
     */
  for (int i = 0; i < 9; i++)
    cov[i / 3 * 6 + (i % 3 + 3)] = P_(i / 3 * N_STATE + (i % 3 + 6));
  /**
     *\return <cov>{18 19 20 24 25 26 30 31 32}
     * \param <P_> {150 151 152 175 176 177 200 201 202}
     */
  for (int i = 0; i < 9; i++)
    cov[(i / 3 + 3) * 6 + i % 3] = P_((i / 3 + 6) * N_STATE + i % 3);
  /**
     *\return <cov>{21 22 23 27 28 29 33 34 35}
     * \param <P_> {156 157 158 181 182 183 206 207 208}
     * 
     */
  for (int i = 0; i < 9; i++)
    cov[(i / 3 + 3) * 6 + (i % 3 + 3)] = P_((i / 3 + 6) * N_STATE + (i % 3 + 6));
}
///将eigen格式的p,q转为ros message格式
///拿到p,q的协方差
void State::toPoseMsg(geometry_msgs::PoseWithCovarianceStamped & pose)
{
  eigen_conversions::vector3dToPoint(p_, pose.pose.pose.position);
  eigen_conversions::quaternionToMsg(q_, pose.pose.pose.orientation);
  getPoseCovariance(pose.pose.covariance); 
}
///将eigen格式的p,q,v,转成ExtState格式
void State::toExtStateMsg(sensor_fusion_comm::ExtState & state)
{
  eigen_conversions::vector3dToPoint(p_, state.pose.position);
  eigen_conversions::quaternionToMsg(q_, state.pose.orientation);
  eigen_conversions::vector3dToPoint(v_, state.velocity);
}
///拿到28维误差状态量
void State::toStateMsg(sensor_fusion_comm::DoubleArrayStamped & state)
{
  state.data[0] = p_[0];
  state.data[1] = p_[1];
  state.data[2] = p_[2];
  state.data[3] = v_[0];
  state.data[4] = v_[1];
  state.data[5] = v_[2];
  state.data[6] = q_.w();
  state.data[7] = q_.x();
  state.data[8] = q_.y();
  state.data[9] = q_.z();
  state.data[10] = b_w_[0];
  state.data[11] = b_w_[1];
  state.data[12] = b_w_[2];
  state.data[13] = b_a_[0];
  state.data[14] = b_a_[1];
  state.data[15] = b_a_[2];
  state.data[16] = L_;
  state.data[17] = q_wv_.w();
  state.data[18] = q_wv_.x();
  state.data[19] = q_wv_.y();
  state.data[20] = q_wv_.z();
  state.data[21] = q_ci_.w();
  state.data[22] = q_ci_.x();
  state.data[23] = q_ci_.y();
  state.data[24] = q_ci_.z();
  state.data[25] = p_ci_[0];
  state.data[26] = p_ci_[1];
  state.data[27] = p_ci_[2];
}

}; // end namespace ssf_core
