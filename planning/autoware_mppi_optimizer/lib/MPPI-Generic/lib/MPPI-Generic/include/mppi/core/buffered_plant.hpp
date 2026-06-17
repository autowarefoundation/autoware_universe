//
// Created by jason on 7/20/21.
//

#ifndef MPPIGENERIC_BUFFERED_PLANT_H
#define MPPIGENERIC_BUFFERED_PLANT_H

#include "buffer.hpp"
#include "base_plant.hpp"

template <class CONTROLLER_T>
class BufferedPlant : public BasePlant<CONTROLLER_T>
{
public:
  using s_array = typename BasePlant<CONTROLLER_T>::s_array;
  using c_array = typename BasePlant<CONTROLLER_T>::c_array;

  using buffer_trajectory = typename BasePlant<CONTROLLER_T>::buffer_trajectory;

  BufferedPlant(std::shared_ptr<CONTROLLER_T> controller, int hz, int opt_stide)
    : BasePlant<CONTROLLER_T>(controller, hz, opt_stide)
  {
  }

  void updateExtraValue(const std::string& name, float value, double time)
  {
    buffer_.updateExtraValue(name, value, time);
  }

  void updateControls(c_array& control, double time)
  {
    buffer_.updateControls(control, time);
  }

  void updateOdometry(Eigen::Vector3f& pos, Eigen::Quaternionf& quat, Eigen::Vector3f& vel, Eigen::Vector3f& omega,
                      double time)
  {
    buffer_.updateOdometry(pos, quat, vel, omega, time);

    /**
     * Uses the most recent odometry information
     * If any other sources are more delayed it uses the most recent value
     * If other sources are more recent it gets interpolated to odom time
     */
    std::map<std::string, float> smoothed = buffer_.getInterpState(time);
    s_array state = this->controller_->model_->stateFromMap(smoothed);
    BasePlant<CONTROLLER_T>::updateState(state, time);
  }

  std::map<std::string, float> getInterpState(double time)
  {
    return buffer_.getInterpState(time);
  }

  bool updateParameters()
  {
    // removes extra values from the buffer
    double time = this->getStateTime();
    buffer_.cleanBuffers(time, buffer_time_horizon_);
    return BasePlant<CONTROLLER_T>::updateParameters();
  }

  buffer_trajectory getSmoothedBuffer(double latest_time)
  {
    return buffer_.getSmoothedBuffer(latest_time, buffer_tau_, buffer_dt_);
  }

  void cleanBuffers(double time)
  {
    buffer_.cleanBuffers(time, buffer_time_horizon_);
  }

  void clearBuffers()
  {
    buffer_.clearBuffers();
  }

  double getBufferDt() const
  {
    return buffer_dt_;
  }

  void setBufferDt(const double buff_dt)
  {
    buffer_dt_ = buff_dt;
  }

protected:
  Buffer<typename CONTROLLER_T::TEMPLATED_DYNAMICS> buffer_;

  double buffer_time_horizon_ = 2.0;  // how long to store values in the buffer
  double buffer_tau_ = 1.0;           // how in history to create well sampled positions from
  double buffer_dt_ = 0.02;           // the spacing between well sampled buffer positions
};

#endif  // MPPIGENERIC_BUFFERED_PLANT_H
