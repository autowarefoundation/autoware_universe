#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <mppi/utils/test_helper.h>
#include <random>
#include <algorithm>
#include <numeric>

#include <mppi/core/buffered_plant.hpp>
#include <mppi/instantiations/cartpole_mppi/cartpole_mppi.cuh>
#include <mppi_test/mock_classes/mock_dynamics.h>
#include <mppi_test/mock_classes/mock_controller.h>
#include <mppi_test/mock_classes/mock_costs.h>
#include <mppi_test/mock_classes/mock_sampler.h>

const double precision = 1e-6;
const float DT = DT;

template <class CONTROLLER_T>
class TestPlant : public BufferedPlant<CONTROLLER_T>
{
public:
  double time_ = 0.0;

  double avgDurationMs_ = 0;
  double avgTickDuration_ = 0;
  double avgSleepTime_ = 0;

  using c_array = typename CONTROLLER_T::control_array;
  using c_traj = typename CONTROLLER_T::control_trajectory;

  using s_array = typename CONTROLLER_T::state_array;
  using s_traj = typename CONTROLLER_T::state_trajectory;

  using DYN_T = typename CONTROLLER_T::TEMPLATED_DYNAMICS;
  using DYN_PARAMS_T = typename DYN_T::DYN_PARAMS_T;
  using COST_T = typename CONTROLLER_T::TEMPLATED_COSTS;
  using COST_PARAMS_T = typename COST_T::COST_PARAMS_T;
  double timestamp_;
  double loop_speed_;

  using buffer_trajectory = typename BufferedPlant<CONTROLLER_T>::buffer_trajectory;

  TestPlant(std::shared_ptr<MockController> controller, double buffer_time_horizon = 2.0, int hz = 20,
            int opt_stride = 1)
    : BufferedPlant<CONTROLLER_T>(controller, hz, opt_stride)
  {
    this->buffer_time_horizon_ = 2.0;
    this->buffer_tau_ = 0.2;
    this->buffer_dt_ = 0.02;
    controller->setDt(this->buffer_dt_);
  }

  void pubControl(const c_array& u) override
  {
  }

  void pubNominalState(const s_array& s) override
  {
  }

  void pubFreeEnergyStatistics(MPPIFreeEnergyStatistics& fe_stats) override
  {
  }

  void incrementTime()
  {
    time_ += DT;
  }

  int checkStatus() override
  {
    return 1;
  }

  double getCurrentTime() override
  {
    auto current_time = std::chrono::system_clock::now();
    auto duration_in_seconds = std::chrono::duration<double>(current_time.time_since_epoch());
    return duration_in_seconds.count();
  }

  double getPoseTime() override
  {
    return time_;
  }

  void setStateTimeToPoseTime()
  {
    this->state_time_ = time_;
  }

  // accessors for protected members
  std::list<BufferMessage<Eigen::Vector3f>> getPrevPositionList()
  {
    return this->buffer_.getPrevPositionList();
  }
  std::list<BufferMessage<Eigen::Quaternionf>> getPrevQuaternionList()
  {
    return this->buffer_.getPrevQuaternionList();
  }
  std::list<BufferMessage<Eigen::Vector3f>> getPrevVelocityList()
  {
    return this->buffer_.getPrevVelocityList();
  }
  std::list<BufferMessage<Eigen::Vector3f>> getPrevOmegaList()
  {
    return this->buffer_.getPrevOmegaList();
  }
  std::list<BufferMessage<c_array>> getPrevControlList()
  {
    return this->buffer_.getPrevControlList();
  }
  std::map<std::string, std::list<BufferMessage<float>>> getPrevExtraList()
  {
    return this->buffer_.getPrevExtraList();
  }
  double getBufferTimeHorizon()
  {
    return this->buffer_time_horizon_;
  }
  double getBufferTau()
  {
    return this->buffer_tau_;
  }
  void setLastUsedUpdateTime(double time)
  {
    this->last_used_state_update_time_ = time;
  }
};

typedef TestPlant<MockController> MockTestPlant;

class BufferedPlantTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mockController = std::make_shared<MockController>();
    // EXPECT_CALL(*mockController, getDt()).Times(1);
    EXPECT_CALL(*mockController, getDt()).WillRepeatedly(testing::Return(DT));
    mockController->cost_ = &mockCost;
    mockController->model_ = &mockDynamics;
    mockController->fb_controller_ = &mockFeedback;
    mockController->sampler_ = &mockSamplingDistribution;

    EXPECT_CALL(*mockController->cost_, getParams()).Times(1);
    EXPECT_CALL(*mockController->model_, getParams()).Times(1);

    EXPECT_CALL(mockDynamics, freeCudaMem()).Times(1);
    EXPECT_CALL(mockCost, freeCudaMem()).Times(1);
    EXPECT_CALL(mockSamplingDistribution, freeCudaMem()).Times(1);
    EXPECT_CALL(mockFeedback, freeCudaMem()).Times(1);

    plant = std::make_shared<MockTestPlant>(mockController);

    generator = std::default_random_engine(7.0);
    distribution = std::normal_distribution<float>(0.0, 100.0);
  }

  void TearDown() override
  {
    plant = nullptr;
    mockController = nullptr;
  }

  MockSamplingDistribution mockSamplingDistribution;
  MockDynamics mockDynamics;
  MockCost mockCost;
  MockFeedback mockFeedback;
  std::shared_ptr<MockController> mockController;
  std::shared_ptr<MockTestPlant> plant;

  std::default_random_engine generator;
  std::normal_distribution<float> distribution;
};

TEST_F(BufferedPlantTest, Constructor)
{
  auto prev_pos = plant->getPrevPositionList();
  EXPECT_EQ(prev_pos.size(), 0);
  auto prev_quat = plant->getPrevQuaternionList();
  EXPECT_EQ(prev_quat.size(), 0);
  auto prev_vel = plant->getPrevVelocityList();
  EXPECT_EQ(prev_vel.size(), 0);
  auto prev_omega = plant->getPrevOmegaList();
  EXPECT_EQ(prev_omega.size(), 0);
  auto prev_control = plant->getPrevControlList();
  EXPECT_EQ(prev_control.size(), 0);
  auto prev_extra = plant->getPrevExtraList();
  EXPECT_EQ(prev_control.size(), 0);

  EXPECT_FLOAT_EQ(plant->getBufferTimeHorizon(), 2.0);
  EXPECT_FLOAT_EQ(plant->getBufferTau(), 0.2);
  EXPECT_FLOAT_EQ(plant->getBufferDt(), 0.02);
}

TEST_F(BufferedPlantTest, setBufferDt)
{
  double new_buffer_dt = 30.0;
  plant->setBufferDt(new_buffer_dt);
  EXPECT_FLOAT_EQ(plant->getBufferDt(), new_buffer_dt);
}

TEST_F(BufferedPlantTest, interpNew)
{
  Eigen::Vector3f pos = Eigen::Vector3f::Ones();
  Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
  Eigen::Vector3f vel = Eigen::Vector3f::Random();
  Eigen::Vector3f omega = Eigen::Vector3f::Random();

  MockDynamics::state_array state = MockDynamics::state_array::Random();

  EXPECT_CALL(mockDynamics, stateFromMap(testing::_)).Times(2).WillRepeatedly(testing::Return(state));
  // Controls never calculated so no calls to controller in updateState()
  EXPECT_CALL(*mockController, getDt()).Times(0);
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_)).Times(0);

  plant->setLastUsedUpdateTime(0);
  plant->updateOdometry(pos, quat, vel, omega, 0.0);
  plant->updateOdometry(pos, quat, vel, omega, 1.0);

  std::map<std::string, float> result_state = plant->getInterpState(10);
  EXPECT_FLOAT_EQ(result_state["OMEGA_X"], omega.x());
  EXPECT_FLOAT_EQ(result_state["OMEGA_Y"], omega.y());
  EXPECT_FLOAT_EQ(result_state["OMEGA_Z"], omega.z());
}

TEST_F(BufferedPlantTest, updateControls)
{
  MockDynamics::control_array u = MockDynamics::control_array::Zero();
  auto prev_control = plant->getPrevControlList();

  for (int i = 0; i < 20; i++)
  {
    u = MockDynamics::control_array::Ones() * 0.2 * i;
    plant->updateControls(u, 0.2 * i);
    prev_control = plant->getPrevControlList();
    EXPECT_EQ(prev_control.size(), i + 1);
  }
  for (int i = 0; i < 20; i++)
  {
    u = MockDynamics::control_array::Ones() * (0.2 * i + 0.1);
    plant->updateControls(u, 0.2 * i + 0.1);
    prev_control = plant->getPrevControlList();
    EXPECT_EQ(prev_control.size(), i + 21);
  }
  prev_control = plant->getPrevControlList();
  EXPECT_EQ(prev_control.size(), 40);

  double time = 0;
  for (auto it = prev_control.begin(); it != prev_control.end(); it++, time += 0.1)
  {
    EXPECT_FLOAT_EQ(it->time, time);
    for (int i = 0; i < MockDynamics::CONTROL_DIM; i++)
    {
      EXPECT_FLOAT_EQ(it->data(i), time);
    }
  }

  plant->time_ = 4.0;
  plant->setStateTimeToPoseTime();
  plant->updateParameters();
  prev_control = plant->getPrevControlList();
  EXPECT_EQ(prev_control.size(), 20);
}

TEST_F(BufferedPlantTest, updateControlsRandom)
{
  MockDynamics::control_array u = MockDynamics::control_array::Random();
  auto prev_control = plant->getPrevControlList();

  for (int i = 0; i < 1000; i++)
  {
    plant->updateControls(u, distribution(generator));
  }
  prev_control = plant->getPrevControlList();
  EXPECT_EQ(prev_control.size(), 1000);

  std::vector<double> times(1000);

  int index = 0;
  for (auto it = prev_control.begin(); it != prev_control.end(); it++, index++)
  {
    times[index] = it->time;
  }

  EXPECT_TRUE(std::is_sorted(times.begin(), times.end()));
}

// TEST_F(BufferedPlantTest, updateControlsInterp)
// {
//   MockDynamics::control_array u = MockDynamics::control_array::Zero();
//   std::list<BufferMessage<MockDynamics::control_array>> prev_control = plant->getPrevControlList();
//
//   for (int i = 0; i < 21; i++)
//   {
//     u = MockDynamics::control_array::Ones() * 0.2 * i;
//     plant->updateControls(u, 0.2 * i);
//     prev_control = plant->getPrevControlList();
//     EXPECT_EQ(prev_control.size(), i + 1);
//   }
//   prev_control = plant->getPrevControlList();
//   EXPECT_EQ(prev_control.size(), 21);
//
//   for (double t = -2.0; t < 6.0; t += 0.01)
//   {
//     MockDynamics::control_array u_interp = MockTestPlant::interp<MockDynamics::control_array>(prev_control, t);
//     if (t < 0)
//     {
//       for (int i = 0; i < MockDynamics::CONTROL_DIM; i++)
//       {
//         EXPECT_NEAR(u_interp(i), 0, precision) << "at time " << t;
//       }
//     }
//     else if (t > 4.0)
//     {
//       for (int i = 0; i < MockDynamics::CONTROL_DIM; i++)
//       {
//         EXPECT_NEAR(u_interp(i), 4.0, precision) << "at time " << t;
//       }
//     }
//     else
//     {
//       for (int i = 0; i < MockDynamics::CONTROL_DIM; i++)
//       {
//         EXPECT_NEAR(u_interp(i), t, precision) << "at time " << t;
//       }
//     }
//   }
// }

TEST_F(BufferedPlantTest, extraValues)
{
  auto extra_info = plant->getPrevExtraList();
  for (int i = 0; i < 20; i++)
  {
    plant->updateExtraValue("steering_angle", 0.2 * i, 0.2 * i);
    plant->updateExtraValue("steering_vel", 0.2 * i, 0.2 * i);
    extra_info = plant->getPrevExtraList();
    EXPECT_EQ(extra_info.size(), 2);
    EXPECT_EQ(extra_info["steering_angle"].size(), i + 1);
    EXPECT_EQ(extra_info["steering_vel"].size(), i + 1);
  }
  for (int i = 0; i < 20; i++)
  {
    plant->updateExtraValue("steering_angle", 0.2 * i + 0.1, 0.2 * i + 0.1);
    plant->updateExtraValue("steering_vel", 0.2 * i + 0.1, 0.2 * i + 0.1);
    extra_info = plant->getPrevExtraList();
    EXPECT_EQ(extra_info.size(), 2);
    EXPECT_EQ(extra_info["steering_angle"].size(), i + 21);
    EXPECT_EQ(extra_info["steering_vel"].size(), i + 21);
  }
  extra_info = plant->getPrevExtraList();
  EXPECT_EQ(extra_info.size(), 2);
  EXPECT_EQ(extra_info["steering_angle"].size(), 40);
  EXPECT_EQ(extra_info["steering_vel"].size(), 40);

  for (auto list_it = extra_info.begin(); list_it != extra_info.end(); list_it++)
  {
    double time = 0;
    for (auto it = list_it->second.begin(); it != list_it->second.end(); it++, time += 0.1)
    {
      EXPECT_FLOAT_EQ(it->time, time);
      for (int i = 0; i < MockDynamics::CONTROL_DIM; i++)
      {
        EXPECT_FLOAT_EQ(it->data, time);
      }
    }
  }

  plant->cleanBuffers(4.0);
  extra_info = plant->getPrevExtraList();
  EXPECT_EQ(extra_info.size(), 2);
  EXPECT_EQ(extra_info["steering_angle"].size(), 20);
  EXPECT_EQ(extra_info["steering_vel"].size(), 20);
}

TEST_F(BufferedPlantTest, updateOdometry)
{
  Eigen::Vector3f pos = Eigen::Vector3f::Ones();
  Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
  Eigen::Vector3f vel = Eigen::Vector3f::Ones();
  Eigen::Vector3f omega = Eigen::Vector3f::Ones();

  MockDynamics::state_array state = MockDynamics::state_array::Random();

  EXPECT_CALL(mockDynamics, stateFromMap(testing::_)).Times(2).WillRepeatedly(testing::Return(state));
  // Controls never calculated so no calls to controller in updateState()
  EXPECT_CALL(*mockController, getDt()).Times(0);
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_)).Times(0);

  plant->setLastUsedUpdateTime(0.0);
  plant->updateOdometry(pos, quat, vel, omega, 0.0);

  auto prev_pos = plant->getPrevPositionList();
  auto prev_quat = plant->getPrevQuaternionList();
  auto prev_vel = plant->getPrevVelocityList();
  auto prev_omega = plant->getPrevOmegaList();

  EXPECT_EQ(prev_pos.size(), 1);
  EXPECT_EQ(prev_quat.size(), 1);
  EXPECT_EQ(prev_vel.size(), 1);
  EXPECT_EQ(prev_omega.size(), 1);
  MockDynamics::state_array result_state = plant->getState();
  EXPECT_LT((state - result_state).norm(), 1e-8);

  pos = Eigen::Vector3f::Ones() * 3;
  vel = Eigen::Vector3f::Ones() * 4;
  omega = Eigen::Vector3f::Ones() * 5;
  quat = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());

  plant->updateOdometry(pos, quat, vel, omega, 1.0);

  prev_pos = plant->getPrevPositionList();
  prev_quat = plant->getPrevQuaternionList();
  prev_vel = plant->getPrevVelocityList();
  prev_omega = plant->getPrevOmegaList();

  EXPECT_EQ(prev_pos.size(), 2);
  EXPECT_EQ(prev_quat.size(), 2);
  EXPECT_EQ(prev_vel.size(), 2);
  EXPECT_EQ(prev_omega.size(), 2);
  result_state = plant->getState();
  EXPECT_LT((state - result_state).norm(), 1e-8);

  std::map<std::string, float> interp = plant->getInterpState(0.5);

  EXPECT_FLOAT_EQ(interp.at("POS_X"), 2);
  EXPECT_FLOAT_EQ(interp.at("POS_Y"), 2);
  EXPECT_FLOAT_EQ(interp.at("POS_Z"), 2);

  EXPECT_FLOAT_EQ(interp.at("Q_W"), 0.92387962);
  EXPECT_FLOAT_EQ(interp.at("Q_X"), 0.0);
  EXPECT_FLOAT_EQ(interp.at("Q_Y"), 0.0);
  EXPECT_FLOAT_EQ(interp.at("Q_Z"), 0.3826834);

  EXPECT_FLOAT_EQ(interp.at("VEL_X"), 2.5);
  EXPECT_FLOAT_EQ(interp.at("VEL_Y"), 2.5);
  EXPECT_FLOAT_EQ(interp.at("VEL_Z"), 2.5);

  EXPECT_FLOAT_EQ(interp.at("OMEGA_X"), 3);
  EXPECT_FLOAT_EQ(interp.at("OMEGA_Y"), 3);
  EXPECT_FLOAT_EQ(interp.at("OMEGA_Z"), 3);
}

TEST_F(BufferedPlantTest, getInterpState)
{
  Eigen::Vector3f pos = Eigen::Vector3f::Ones();
  Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
  Eigen::Vector3f vel = Eigen::Vector3f::Ones();
  Eigen::Vector3f omega = Eigen::Vector3f::Ones();
  MockDynamics::control_array u = MockDynamics::control_array::Ones();

  MockDynamics::state_array state = MockDynamics::state_array::Random();

  plant->setLastUsedUpdateTime(0.0);
  EXPECT_CALL(mockDynamics, stateFromMap(testing::_)).Times(2).WillRepeatedly(testing::Return(state));
  // Controls never calculated so no calls to controller in updateState()
  EXPECT_CALL(*mockController, getDt()).Times(0);
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_)).Times(0);

  plant->updateOdometry(pos, quat, vel, omega, 0.0);
  plant->updateControls(u, 0.0);
  plant->updateExtraValue("steering_angle", 1, 0);
  plant->updateExtraValue("steering_vel", 1, 0);

  pos = Eigen::Vector3f::Ones() * 2;
  vel = Eigen::Vector3f::Ones() * 2;
  omega = Eigen::Vector3f::Ones() * 2;
  quat = Eigen::AngleAxisf(M_PI_2f32, Eigen::Vector3f::UnitZ());
  u = MockDynamics::control_array::Ones() * 2;

  plant->updateOdometry(pos, quat, vel, omega, 1.0);
  plant->updateControls(u, 1.0);
  plant->updateExtraValue("steering_angle", 2, 1.0);
  plant->updateExtraValue("steering_vel", 2, 1.0);

  std::map<std::string, float> map = plant->getInterpState(0.5);

  EXPECT_EQ(map.size(), 19);

  EXPECT_FLOAT_EQ(map.at("POS_X"), 1.5);
  EXPECT_FLOAT_EQ(map.at("POS_Y"), 1.5);
  EXPECT_FLOAT_EQ(map.at("POS_Z"), 1.5);

  EXPECT_FLOAT_EQ(map.at("Q_W"), 0.92387962);
  EXPECT_FLOAT_EQ(map.at("Q_X"), 0.0);
  EXPECT_FLOAT_EQ(map.at("Q_Y"), 0.0);
  EXPECT_FLOAT_EQ(map.at("Q_Z"), 0.3826834);

  EXPECT_FLOAT_EQ(map.at("ROLL"), 0);
  EXPECT_FLOAT_EQ(map.at("PITCH"), 0);
  EXPECT_FLOAT_EQ(map.at("YAW"), M_PI_4f32);

  EXPECT_FLOAT_EQ(map.at("VEL_X"), 1.5);
  EXPECT_FLOAT_EQ(map.at("VEL_Y"), 1.5);
  EXPECT_FLOAT_EQ(map.at("VEL_Z"), 1.5);

  EXPECT_FLOAT_EQ(map.at("OMEGA_X"), 1.5);
  EXPECT_FLOAT_EQ(map.at("OMEGA_Y"), 1.5);
  EXPECT_FLOAT_EQ(map.at("OMEGA_Z"), 1.5);

  EXPECT_FLOAT_EQ(map.at("CONTROL_0"), 1.5);

  EXPECT_FLOAT_EQ(map.at("steering_angle"), 1.5);
  EXPECT_FLOAT_EQ(map.at("steering_vel"), 1.5);
}

TEST_F(BufferedPlantTest, getInterpBuffer)
{
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega = Eigen::Vector3f::Zero();
  MockDynamics::control_array u = MockDynamics::control_array::Zero();

  MockDynamics::state_array state = MockDynamics::state_array::Random();

  plant->setLastUsedUpdateTime(0.0);
  EXPECT_CALL(mockDynamics, stateFromMap(testing::_)).Times(2).WillRepeatedly(testing::Return(state));
  // Controls never calculated so no calls to controller in updateState()
  EXPECT_CALL(*mockController, getDt()).Times(0);
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_)).Times(0);

  plant->updateOdometry(pos, quat, vel, omega, 0.0);
  plant->updateControls(u, 0.0);
  plant->updateExtraValue("steering_angle", 0, 0);
  plant->updateExtraValue("steering_vel", 0, 0);

  pos = Eigen::Vector3f::Ones();
  vel = Eigen::Vector3f::Ones();
  omega = Eigen::Vector3f::Ones();
  quat = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  u = MockDynamics::control_array::Ones();

  plant->updateOdometry(pos, quat, vel, omega, 1.0);
  plant->updateControls(u, 1.0);
  plant->updateExtraValue("steering_angle", 1, 1.0);
  plant->updateExtraValue("steering_vel", 1, 1.0);

  MockTestPlant::buffer_trajectory buffer = plant->getSmoothedBuffer(1.0);

  EXPECT_EQ(buffer.size(), 19);
  EXPECT_EQ(buffer.at("POS_X").size(), 11);

  for (int t = 0; t < 11; t++)
  {
    double time = 0.8 + t * 0.02;
    EXPECT_FLOAT_EQ(buffer.at("POS_X")(t), time) << "at time " << t << " " << time;
    EXPECT_FLOAT_EQ(buffer.at("POS_Y")(t), time) << "at time " << t << " " << time;
    EXPECT_FLOAT_EQ(buffer.at("POS_Z")(t), time) << "at time " << t << " " << time;

    EXPECT_FLOAT_EQ(buffer.at("VEL_X")(t), time) << "at time " << t << " " << time;
    EXPECT_FLOAT_EQ(buffer.at("VEL_Y")(t), time) << "at time " << t << " " << time;
    EXPECT_FLOAT_EQ(buffer.at("VEL_Z")(t), time) << "at time " << t << " " << time;

    EXPECT_FLOAT_EQ(buffer.at("OMEGA_X")(t), time) << "at time " << t << " " << time;
    EXPECT_FLOAT_EQ(buffer.at("OMEGA_Y")(t), time) << "at time " << t << " " << time;
    EXPECT_FLOAT_EQ(buffer.at("OMEGA_Z")(t), time) << "at time " << t << " " << time;

    EXPECT_FLOAT_EQ(buffer.at("ROLL")(t), 0);
    EXPECT_FLOAT_EQ(buffer.at("PITCH")(t), 0);

    EXPECT_FLOAT_EQ(buffer.at("CONTROL_0")(t), time) << "at time " << t << " " << time;
    EXPECT_NEAR(buffer.at("steering_angle")(t), time, precision) << "at time " << t << " " << time;
    EXPECT_NEAR(buffer.at("steering_vel")(t), time, precision) << "at time " << t << " " << time;
  }
}
