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

#ifndef TEMPLATE_MEASUREMENTS_H
#define TEMPLATE_MEASUREMENTS_H

#include <ros/ros.h>
#include <ssf_core/measurement.h>
#include "template_sensor.h"

class TemplateMeasurements : public ssf_core::Measurements
{
public:
  TemplateMeasurements()
  {
    addHandler(new TemplateSensorHandler(this));

    ros::NodeHandle pnh("~");
    pnh.param("init/p_ci/x", p_ci_[0], 0.0);
    pnh.param("init/p_ci/y", p_ci_[1], 0.0);
    pnh.param("init/p_ci/z", p_ci_[2], 0.0);

    pnh.param("init/q_ci/w", q_ci_.w(), 1.0);
    pnh.param("init/q_ci/x", q_ci_.x(), 0.0);
    pnh.param("init/q_ci/y", q_ci_.y(), 0.0);
    pnh.param("init/q_ci/z", q_ci_.z(), 0.0);
    q_ci_.normalize();
    
  }

private:

  Eigen::Matrix<double, 3, 1> p_ci_; ///< initial distance sensor-IMU
  Eigen::Quaternion<double> q_ci_; ///< initial rotation sensor-IMU
  Eigen::Quaternion<double> q_wv_; ///< initial rotation wolrd-vision

  void init(double scale)
  {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m;
    Eigen::Quaternion<double> q,q_wv_;
    ssf_core::SSF_Core::ErrorStateCov P;

    // init values
	g << 0, 0, 9.81;	/// gravity
	b_w << 0,0,0;		/// bias gyroscopes
	b_a << 0,0,0;		/// bias accelerometer

	v << 0,0,0;			/// robot velocity (IMU centered)
	w_m << 0,0,0;		/// initial angular velocity
	a_m =g;				/// initial acceleration
	q_wv_.setIdentity();
        P.setZero(); // error state covariance; if zero, a default initialization in ssf_core is used
    // initialization callback when the user hits init_filter
    // in the dynamic reconfigure gui. This initializes the states based on the
    // current update-sensor readings.    

    // states to be initialized:
    // g: gravity vector
    // a_m: initial acceleration readings (usually set a_m=g)
    // w_m: initial gyro readings (usually set w_m=g)
    // p_ci, q_ci: 6DoF transformation between sensor and IMU (already initialized by the constructor)
    // P: initial error state covariance matrix (use P.setZero(); to use default initialization)
    // q_wv: attitude drift state between sensor reference frame and world frame (uausally use q_wv.setIdentity();)
    // b_a: bias accelerometers (set to zero)
    // b_w: bias gyros (set to zero)
    // q: initial IMU attitude (you may use: q = (q_ci_ * q_cv_.conjugate() * q_wv).conjugate(); q_cv is the current sensor attitude reading
    // v: initial velocity (set to zero)
    // p: initial IMU position (you may use: q_wv.conjugate().toRotationMatrix() * p_vc_ / scale - q.toRotationMatrix() * p_ci_; p_cv is the current sensor position reading
    // 
    // check if we have already input from the measurement sensor
    if (p_vc_.norm() == 0)
      ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
    if ((q_cv_.norm() == 1) & (q_cv_.w() == 1))
      ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

    q = (q_ci_ * q_cv_.conjugate() * q_wv_).conjugate();//q_wi_
    q.normalize();
    //这个式子是测量模型位置部分的转换方程
    p = q_wv_.conjugate().toRotationMatrix() * p_vc_ / scale - q.toRotationMatrix() * p_ci_;

   // call initialization in core
    ssf_core_.initialize(p, v, q, b_w, b_a, scale, q_wv_, P, w_m, a_m, g, q_ci_, p_ci_);

    ROS_INFO_STREAM("filter initialized to: \n" <<
        "position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl <<
        "scale:" << scale << std::endl <<
        "attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl <<
        "p_ci: [" << p_ci_[0] << ", " << p_ci_[1] << ", " << p_ci_[2] << std::endl <<
        "q_ci: (w,x,y,z): [" << q_ci_.w() << ", " << q_ci_.x() << ", " << q_ci_.y() << ", " << q_ci_.z() << "]");
  }
};

#endif /* TEMPLATE_MEASUREMENTS_H */
