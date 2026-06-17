//
// Created by jgibson37 on 2/24/20.
//

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mppi/core/base_plant.hpp>
#include <mppi/instantiations/cartpole_mppi/cartpole_mppi.cuh>
#include <mppi/utils/test_helper.h>
#include <mppi_test/mock_classes/mock_controller.h>

#include <algorithm>
#include <numeric>
#include <random>

const float DT = 0.05;

template <class CONTROLLER_T>
class TestPlant : public BasePlant<CONTROLLER_T>
{
public:
  double time_ = 0.0;

  int pubControlCalled = 0;
  int pubNominalStateCalled = 0;

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

  TestPlant(std::shared_ptr<MockController> controller, int hz = 20, int opt_stride = 1)
    : BasePlant<CONTROLLER_T>(controller, hz, opt_stride)
  {
  }

  void pubControl(const c_array& u) override
  {
    pubControlCalled++;
  }

  void pubNominalState(const s_array& s) override
  {
    pubNominalStateCalled++;
  }

  void pubFreeEnergyStatistics(MPPIFreeEnergyStatistics& fe_stats) override
  {
  }

  void incrementTime()
  {
    time_ += DT;
  }

  void incrementTime(double dt)
  {
    time_ += dt;
  }

  int checkStatus() override
  {
    return 1;
  }

  double getCurrentTime()
  {
    return time_ + 0.3421;
  }

  double getPoseTime()
  {
    return time_;
  }

  // accessors for protected members
  int getNumIter()
  {
    return this->num_iter_;
  }
  double getLastUsedPoseUpdateTime()
  {
    return this->last_used_state_update_time_;
  }
  int getStatus()
  {
    return this->status_;
  }
  bool getDebugMode()
  {
    return this->debug_mode_;
  }
  double getOptimizationDuration()
  {
    return this->optimization_duration_;
  }
  double getOptimizationAvg()
  {
    return this->avg_optimize_time_ms_;
  }
  double getOptimizeLoopDuration()
  {
    return this->optimize_loop_duration_;
  }
  double getLoopAvg()
  {
    return this->avg_loop_time_ms_;
  }
  double getFeedbackDuration()
  {
    return this->feedback_duration_;
  }
  double getFeedbackAvg()
  {
    return this->avg_feedback_time_ms_;
  }
  void setLastTime(double time)
  {
    time_ = time;
  }
  void setLastUsedTime(double time)
  {
    this->last_used_state_update_time_ = time;
  }
  double getSleepTimeAvg()
  {
    return this->avg_sleep_time_ms_;
  }
};

typedef TestPlant<MockController> MockTestPlant;

class BasePlantTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    EXPECT_CALL(mockCost, getParams()).Times(1);
    EXPECT_CALL(mockDynamics, getParams()).Times(1);

    EXPECT_CALL(mockDynamics, freeCudaMem()).Times(1);
    EXPECT_CALL(mockCost, freeCudaMem()).Times(1);
    EXPECT_CALL(mockSampler, freeCudaMem()).Times(1);
    EXPECT_CALL(mockFeedback, freeCudaMem()).Times(1);

    mockController = std::make_shared<MockController>();
    mockController->setDt(DT);
    EXPECT_CALL(*mockController, getDt()).WillRepeatedly(testing::Return(DT));
    mockController->cost_ = &mockCost;
    mockController->model_ = &mockDynamics;
    mockController->sampler_ = &mockSampler;
    mockController->fb_controller_ = &mockFeedback;

    plant = std::make_shared<MockTestPlant>(mockController);
  }

  void TearDown() override
  {
  }
  MockDynamics mockDynamics;
  MockCost mockCost;
  MockFeedback mockFeedback;
  MockSamplingDistribution mockSampler;
  std::shared_ptr<MockController> mockController;
  std::shared_ptr<MockTestPlant> plant;

  const float SMALL_TIME_MS = 8;
};

TEST_F(BasePlantTest, Constructor)
{
  EXPECT_EQ(plant->controller_, mockController);
  EXPECT_EQ(plant->getHz(), 20);
  EXPECT_EQ(plant->getTargetOptimizationStride(), 1);
  EXPECT_EQ(plant->getNumIter(), 0);
  EXPECT_EQ(plant->getLastUsedPoseUpdateTime(), 0);
  EXPECT_EQ(plant->getStatus(), 1);
  EXPECT_EQ(mockController->getFeedbackEnabled(), false);
  EXPECT_EQ(plant->hasNewCostParams(), false);
  EXPECT_EQ(plant->hasNewDynamicsParams(), false);
}

TEST_F(BasePlantTest, getAndSetState)
{
  // check initial state is zerod

  MockController::state_array state = plant->getState();
  for (int i = 0; i < 1; i++)
  {
    EXPECT_EQ(state(i), 0.0);
  }

  MockController::state_array new_state;
  for (int i = 0; i < 1; i++)
  {
    new_state(i) = i;
  }
  plant->setState(new_state);
  state = plant->getState();
  for (int i = 0; i < 1; i++)
  {
    EXPECT_EQ(state(i), i);
  }
}

TEST_F(BasePlantTest, getSetOptimizationStride)
{
  int optimization_stride = plant->getTargetOptimizationStride();

  EXPECT_EQ(optimization_stride, 1);

  plant->setTargetOptimizationStride(5);
  optimization_stride = plant->getTargetOptimizationStride();

  EXPECT_EQ(optimization_stride, 5);
}

TEST_F(BasePlantTest, getSetDynamicsParams)
{
  EXPECT_EQ(plant->hasNewDynamicsParams(), false);
  MockTestPlant::DYN_PARAMS_T params;

  params.test = 3;

  plant->setDynamicsParams(params);
  EXPECT_EQ(plant->hasNewDynamicsParams(), true);

  MockTestPlant::DYN_PARAMS_T new_params = plant->getNewDynamicsParams();
  EXPECT_EQ(plant->hasNewDynamicsParams(), false);
  EXPECT_EQ(new_params.test, params.test);
}

TEST_F(BasePlantTest, getSetCostParams)
{
  EXPECT_EQ(plant->hasNewCostParams(), false);

  MockTestPlant::COST_PARAMS_T params;
  params.test = 100;

  plant->setCostParams(params);
  EXPECT_EQ(plant->hasNewCostParams(), true);

  auto new_params = plant->getNewCostParams();
  EXPECT_EQ(plant->hasNewCostParams(), false);
  EXPECT_EQ(params.test, new_params.test);
}

TEST_F(BasePlantTest, updateParametersAllFalse)
{
  EXPECT_CALL(mockCost, setParams(testing::_)).Times(0);
  EXPECT_CALL(mockDynamics, setParams(testing::_)).Times(0);

  plant->updateParameters();
}

TEST_F(BasePlantTest, updateParametersAllTrue)
{
  EXPECT_CALL(mockCost, setParams(testing::_)).Times(1);
  EXPECT_CALL(mockDynamics, setParams(testing::_)).Times(1);

  plant->setDebugMode(true);
  plant->setDynamicsParams(MockDynamics::DYN_PARAMS_T());
  plant->setCostParams(MockCost::COST_PARAMS_T());

  plant->updateParameters();
}

TEST_F(BasePlantTest, updateStateOutsideTimeTest)
{
  plant->setLastTime(0);

  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_)).Times(0);

  MockController::state_array state = MockController::state_array::Zero();
  plant->updateState(state, mockController->getDt() * mockController->getNumTimesteps() + 0.01);
  EXPECT_EQ(plant->getState(), state);

  plant->setLastTime(100);
  plant->updateState(state, 99.99);
  EXPECT_EQ(plant->getState(), state);
  EXPECT_EQ(plant->pubControlCalled, 0);
  EXPECT_EQ(plant->pubNominalStateCalled, 0);
}

TEST_F(BasePlantTest, updateStateTest)
{
  plant->setLastTime(0);

  MockController::state_array state = MockController::state_array::Zero();
  // EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_,
  // testing::_)).Times(0);
  plant->updateState(state, 12);
  EXPECT_EQ(plant->pubControlCalled, 0);
  EXPECT_EQ(plant->pubNominalStateCalled, 0);

  // Calling updateState() should not pub controls when none have been calculated as of yet
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_)).Times(0);
  plant->setLastUsedTime(11);
  plant->updateState(state, 12);
  EXPECT_EQ(plant->getState(), state);
  EXPECT_EQ(plant->pubControlCalled, 0);
  EXPECT_EQ(plant->pubNominalStateCalled, 0);

  // TODO in debug should pub nominal state
}

TEST_F(BasePlantTest, pubControlOnlyAfterControlAreCalculatedTest)
{
  ::testing::Sequence s1;
  // Step 1: calling updateState() before controls are calculated should not call controller->getCurrentControl()
  MockController::state_array state = MockController::state_array::Zero();
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(0)
      .InSequence(s1);
  double curr_time = 0.0;
  plant->updateState(state, curr_time);
  EXPECT_EQ(plant->pubControlCalled, 0);
  EXPECT_EQ(plant->pubNominalStateCalled, 0);
  // ::testing::Mock::VerifyAndClearExpectations(mockController.get());

  // Step 2: run control iteration inside plant
  // Create valid outputs from gmock methods to prevent nan detection from triggering
  MockController::control_trajectory valid_control_seq =
      MockController::control_trajectory::Zero(MockDynamics::CONTROL_DIM, NUM_TIMESTEPS);
  MockController::state_trajectory valid_state_seq =
      MockController::state_trajectory::Zero(MockDynamics::STATE_DIM, NUM_TIMESTEPS);
  EXPECT_CALL(*mockController, computeControl(testing::_, testing::_)).Times(1).InSequence(s1);
  EXPECT_CALL(*mockController, getControlSeq()).Times(1).WillRepeatedly(testing::Return(valid_control_seq));
  EXPECT_CALL(*mockController, getTargetStateSeq()).Times(1).WillRepeatedly(testing::Return(valid_state_seq));
  // EXPECT_CALL(*mockController, getTargetOutputSeq()).Times(1);
  std::atomic<bool> is_alive(true);
  plant->runControlIteration(&is_alive);

  // Step 3: calling updateState() now should use controller->getCurrentControl()
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(1)
      .InSequence(s1);
  curr_time++;
  plant->updateState(state, curr_time);
  EXPECT_EQ(plant->pubControlCalled, 1);
  EXPECT_EQ(plant->pubNominalStateCalled, 0);
}

TEST_F(BasePlantTest, EnsureReceivingStateCompletesRunControlIterationTest)
{
  std::atomic<bool> is_alive(true);
  // Create valid outputs from gmock methods to prevent nan detection from triggering
  MockController::control_trajectory valid_control_seq =
      MockController::control_trajectory::Zero(MockDynamics::CONTROL_DIM, NUM_TIMESTEPS);
  MockController::state_trajectory valid_state_seq =
      MockController::state_trajectory::Zero(MockDynamics::STATE_DIM, NUM_TIMESTEPS);
  EXPECT_CALL(*mockController, computeControl(testing::_, testing::_)).Times(1);
  EXPECT_CALL(*mockController, getControlSeq()).Times(1).WillRepeatedly(testing::Return(valid_control_seq));
  EXPECT_CALL(*mockController, getTargetStateSeq()).Times(1).WillRepeatedly(testing::Return(valid_state_seq));
  // EXPECT_CALL(*mockController, getTargetOutputSeq()).Times(1);
  std::thread new_thread(&MockTestPlant::runControlIteration, plant.get(), &is_alive);
  // Wait some period of time and then call updateState()
  std::cout << "Wait for new state" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_)).Times(0);
  MockController::state_array state = MockController::state_array::Zero();
  double curr_time = 0.0;
  plant->updateState(state, curr_time);
  std::cout << "State sent" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // is_alive.store(false);
  new_thread.join();
}

TEST_F(BasePlantTest, runControlIterationStoppedTest)
{
  EXPECT_CALL(*mockController, slideControlSequence(testing::_)).Times(0);
  EXPECT_CALL(*mockController, computeControl(testing::_, testing::_)).Times(0);

  std::atomic<bool> is_alive(false);
  plant->runControlIteration(&is_alive);
}

// TODO speed up to make tests run faster
TEST_F(BasePlantTest, runControlIterationDebugFalseNoFeedbackTest)
{
  double init_time = 100;
  plant->setLastTime(init_time);
  MockController::state_array s = MockController::state_array::Zero();
  for (int i = 0; i < 2; i++)
  {
    double wait_ms = 50 * i;

    auto wait_function = [wait_ms](const Eigen::Ref<const MockController::state_array>& state,
                                   int optimization_stride = 0) { usleep(wait_ms * 1e3); };

    int expect_opt_stride = i > 0 ? 1 : 0;

    MockDynamics::control_array control = MockDynamics::control_array::Zero();
    if (i > 0)
    {
      EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
          .Times(1)
          .WillRepeatedly(testing::Return(control));  // called by update state should not be called since time since
                                                      // last
      // opt should be zero on first iteration, should be called on second
    }
    else
    {
      EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
          .Times(0);
    }

    EXPECT_CALL(*mockController, slideControlSequence(expect_opt_stride)).Times(i > 0 ? 1 : 0);
    EXPECT_CALL(*mockController, computeControl(testing::_, testing::_))
        .Times(1)
        .WillRepeatedly(testing::Invoke(wait_function));
    MockController::control_trajectory control_seq = MockController::control_trajectory::Zero();
    EXPECT_CALL(*mockController, getControlSeq()).Times(1).WillRepeatedly(testing::Return(control_seq));
    MockController::state_trajectory state_seq = MockController::state_trajectory::Zero();
    EXPECT_CALL(*mockController, getTargetStateSeq()).Times(1).WillRepeatedly(testing::Return(state_seq));

    EXPECT_CALL(*mockController, computeFeedback(testing::_)).Times(0);
    EXPECT_CALL(*mockController, getFeedbackControl(testing::_, testing::_, testing::_)).Times(0);
    EXPECT_CALL(*mockController, computeFeedbackPropagatedStateSeq()).Times(1);
    EXPECT_CALL(*mockController, calculateSampledStateTrajectories()).Times(0);

    EXPECT_EQ(plant->getDebugMode(), false);

    std::atomic<bool> is_alive(true);
    plant->updateState(s, init_time + i * DT);
    plant->runControlIteration(&is_alive);
    plant->incrementTime();

    EXPECT_EQ(plant->checkStatus(), 1);
    EXPECT_EQ(plant->getStateTraj(), state_seq);
    EXPECT_EQ(plant->getControlTraj(), control_seq);
    MockController::TEMPLATED_FEEDBACK_STATE feedback = plant->getFeedbackState();
    MockController::state_array state = MockController::state_array::Ones();
    for (int j = 0; j < 100; j++)
    {
      // TODO check that feedback is correct
      // auto result = feedback[i] * state;
      // float sum = feedback[i] * state;
      // EXPECT_FLOAT_EQ(result(0), 0);
    }

    // check last pose update
    EXPECT_FLOAT_EQ(plant->getLastUsedPoseUpdateTime(), DT * i + init_time);
    EXPECT_EQ(plant->getNumIter(), i + 1);
    EXPECT_EQ(plant->getLastOptimizationStride(), expect_opt_stride);

    EXPECT_THAT(plant->getOptimizationDuration(),
                testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
    EXPECT_LT(plant->getOptimizationAvg(), wait_ms + SMALL_TIME_MS);
    EXPECT_THAT(plant->getOptimizeLoopDuration(),
                testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
    EXPECT_LT(plant->getLoopAvg(), wait_ms + SMALL_TIME_MS);
    EXPECT_LE(plant->getFeedbackDuration(), SMALL_TIME_MS);
    EXPECT_LE(plant->getFeedbackAvg(), SMALL_TIME_MS);
  }
}

TEST_F(BasePlantTest, runControlIterationDebugFalseFeedbackTest)
{
  EXPECT_CALL(mockFeedback, initTrackingController()).Times(1);
  mockController->initFeedback();

  double init_time = 51789;
  plant->setLastTime(init_time);

  MockController::state_array s = MockController::state_array::Zero();
  for (int i = 0; i < 10; i++)
  {
    double wait_ms = 50 * i;

    auto wait_function = [wait_ms](const Eigen::Ref<const MockController::state_array>& state,
                                   int optimization_stride = 0) { usleep(wait_ms * 1e3); };

    int expect_opt_stride = i > 0 ? 1 : 0;

    MockDynamics::control_array control = MockDynamics::control_array::Zero();
    if (i > 0)
    {
      EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
          .Times(1)
          .WillRepeatedly(testing::Return(control));  // called by update state should not be called since time since
                                                      // last
      // opt should be zero on first iteration, should be called on rest
    }
    else
    {
      EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
          .Times(0);
    }

    EXPECT_CALL(*mockController, slideControlSequence(expect_opt_stride)).Times(i > 0 ? 1 : 0);
    EXPECT_CALL(*mockController, computeControl(testing::_, testing::_))
        .Times(1)
        .WillRepeatedly(testing::Invoke(wait_function));
    MockController::control_trajectory control_seq = MockController::control_trajectory::Zero();
    EXPECT_CALL(*mockController, getControlSeq()).Times(1).WillRepeatedly(testing::Return(control_seq));
    MockController::state_trajectory state_seq = MockController::state_trajectory::Zero();
    EXPECT_CALL(*mockController, getTargetStateSeq()).Times(1).WillRepeatedly(testing::Return(state_seq));

    EXPECT_CALL(*mockController, computeFeedback(testing::_)).Times(1).WillRepeatedly(testing::Invoke(wait_function));
    MockController::TEMPLATED_FEEDBACK_STATE feedback;
    EXPECT_CALL(*mockController, getFeedbackState()).Times(1).WillRepeatedly(testing::Return(feedback));
    EXPECT_CALL(*mockController, computeFeedbackPropagatedStateSeq()).Times(1);
    EXPECT_CALL(*mockController, calculateSampledStateTrajectories()).Times(0);

    EXPECT_EQ(plant->getDebugMode(), false);

    std::atomic<bool> is_alive(true);
    plant->updateState(s, init_time + i * DT);
    plant->runControlIteration(&is_alive);
    plant->incrementTime();

    EXPECT_EQ(plant->checkStatus(), 1);
    EXPECT_EQ(plant->getStateTraj(), state_seq);
    EXPECT_EQ(plant->getControlTraj(), control_seq);
    // EXPECT_EQ(plant->getFeedbackState(), feedback);

    // check last pose update
    EXPECT_FLOAT_EQ(plant->getLastUsedPoseUpdateTime(), DT * i + init_time);
    EXPECT_EQ(plant->getNumIter(), i + 1);
    EXPECT_EQ(plant->getLastOptimizationStride(), expect_opt_stride);

    EXPECT_THAT(plant->getOptimizationDuration(),
                testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
    EXPECT_LT(plant->getOptimizationAvg(), wait_ms + SMALL_TIME_MS);
    EXPECT_THAT(plant->getFeedbackDuration(),
                testing::AllOf(testing::Ge(wait_ms), testing::Le((wait_ms + SMALL_TIME_MS) * 2)));
    // TODO should be range as well
    EXPECT_LT(plant->getFeedbackAvg(), wait_ms + SMALL_TIME_MS);
    EXPECT_THAT(plant->getOptimizeLoopDuration(),
                testing::AllOf(testing::Ge(wait_ms * 2), testing::Le((wait_ms + SMALL_TIME_MS) * 2)));
    EXPECT_LT(plant->getLoopAvg(), (wait_ms + SMALL_TIME_MS) * 2);
  }
}

TEST_F(BasePlantTest, runControlIterationDebugFalseFeedbackAvgTest)
{
  EXPECT_CALL(mockFeedback, initTrackingController()).Times(1);
  mockController->initFeedback();

  double init_time = 51531;
  plant->setLastTime(init_time);
  MockController::state_array s = MockController::state_array::Zero();

  for (int i = 0; i < 10; i++)
  {
    double wait_ms = 50;

    auto wait_function = [wait_ms](const Eigen::Ref<const MockController::state_array>& state,
                                   int optimization_stride = 0) { usleep(wait_ms * 1e3); };

    int expect_opt_stride = i > 0 ? 1 : 0;

    MockDynamics::control_array control = MockDynamics::control_array::Zero();
    if (i > 0)
    {
      EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
          .Times(1)
          .WillRepeatedly(testing::Return(control));  // called by update state should not be called since time since
                                                      // last
      // opt should be zero on first iteration, should be called on rest
    }
    else
    {
      EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
          .Times(0);
    }

    EXPECT_CALL(*mockController, slideControlSequence(expect_opt_stride)).Times(i > 0 ? 1 : 0);
    EXPECT_CALL(*mockController, computeControl(testing::_, testing::_))
        .Times(1)
        .WillRepeatedly(testing::Invoke(wait_function));
    MockController::control_trajectory control_seq = MockController::control_trajectory::Zero();
    EXPECT_CALL(*mockController, getControlSeq()).Times(1).WillRepeatedly(testing::Return(control_seq));
    MockController::state_trajectory state_seq = MockController::state_trajectory::Zero();
    EXPECT_CALL(*mockController, getTargetStateSeq()).Times(1).WillRepeatedly(testing::Return(state_seq));

    EXPECT_CALL(*mockController, computeFeedback(testing::_)).Times(1).WillRepeatedly(testing::Invoke(wait_function));
    MockController::TEMPLATED_FEEDBACK_STATE feedback;
    EXPECT_CALL(*mockController, getFeedbackState()).Times(1).WillRepeatedly(testing::Return(feedback));
    EXPECT_CALL(*mockController, computeFeedbackPropagatedStateSeq()).Times(1);
    EXPECT_CALL(*mockController, calculateSampledStateTrajectories()).Times(0);

    EXPECT_EQ(plant->getDebugMode(), false);

    std::atomic<bool> is_alive(true);
    plant->updateState(s, init_time + i * DT);
    plant->runControlIteration(&is_alive);
    plant->incrementTime();

    EXPECT_EQ(plant->checkStatus(), 1);
    EXPECT_EQ(plant->getStateTraj(), state_seq);
    EXPECT_EQ(plant->getControlTraj(), control_seq);
    // EXPECT_EQ(plant->getFeedbackState(), feedback);

    // check last pose update
    EXPECT_FLOAT_EQ(plant->getLastUsedPoseUpdateTime(), DT * i + init_time);
    EXPECT_EQ(plant->getNumIter(), i + 1);
    EXPECT_EQ(plant->getLastOptimizationStride(), expect_opt_stride);

    EXPECT_THAT(plant->getOptimizationDuration(),
                testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
    EXPECT_THAT(plant->getOptimizationAvg(),
                testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
    EXPECT_THAT(plant->getOptimizeLoopDuration(),
                testing::AllOf(testing::Ge(wait_ms * 2), testing::Le((wait_ms + SMALL_TIME_MS) * 2)));
    EXPECT_THAT(plant->getLoopAvg(),
                testing::AllOf(testing::Ge((wait_ms)*2), testing::Le((wait_ms + SMALL_TIME_MS) * 2)));
    EXPECT_THAT(plant->getFeedbackDuration(),
                testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
    EXPECT_THAT(plant->getFeedbackAvg(), testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  }
}

TEST_F(BasePlantTest, runControlLoopRegular)
{
  EXPECT_CALL(mockFeedback, initTrackingController()).Times(1);
  mockController->initFeedback();

  int hz = plant->getHz();
  double test_duration = 1.0;  // in seconds for how long to run the test

  // setup mock expected calls
  EXPECT_CALL(mockCost, setParams(testing::_)).Times(0);
  EXPECT_CALL(mockDynamics, setParams(testing::_)).Times(0);
  EXPECT_CALL(*mockController, resetControls()).Times(1);

  double wait_s =
      (1.0 / hz) / 3;  // divide by 3 since wait is evenly split across computeFeedback, computeControl, and waiting

  auto wait_function = [wait_s](const Eigen::Ref<const MockController::state_array>& state,
                                int optimization_stride = 0) { usleep(wait_s * 1e6); };
  int iterations = int(std::round((hz * 1.0) / (test_duration)));  // number of times the method will be called

  MockDynamics::control_array control = MockDynamics::control_array::Zero();
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(46)
      .WillRepeatedly(testing::Return(control));  // called by update state should not be called since time since last
  // slide control sequence is skipped on the first iteration
  EXPECT_CALL(*mockController, slideControlSequence(1)).Times(iterations / 2 - 1);
  EXPECT_CALL(*mockController, computeControl(testing::_, testing::_))
      .Times(iterations / 2)
      .WillRepeatedly(testing::Invoke(wait_function));
  MockController::control_trajectory control_seq = MockController::control_trajectory::Zero();
  EXPECT_CALL(*mockController, getControlSeq()).Times(iterations / 2).WillRepeatedly(testing::Return(control_seq));
  MockController::state_trajectory state_seq = MockController::state_trajectory::Zero();
  EXPECT_CALL(*mockController, getTargetStateSeq()).Times(iterations / 2).WillRepeatedly(testing::Return(state_seq));
  EXPECT_CALL(*mockController, computeFeedback(testing::_))
      .Times(iterations / 2)
      .WillRepeatedly(testing::Invoke(wait_function));
  MockController::TEMPLATED_FEEDBACK_STATE feedback;
  EXPECT_CALL(*mockController, getFeedbackState()).Times(iterations / 2).WillRepeatedly(testing::Return(feedback));
  EXPECT_CALL(*mockController, computeFeedbackPropagatedStateSeq()).Times(iterations / 2);
  EXPECT_CALL(*mockController, calculateSampledStateTrajectories()).Times(0);

  int init_time = 78;
  plant->setLastTime(init_time);
  MockDynamics::state_array state = MockDynamics::state_array::Zero();
  plant->updateState(state, init_time);

  std::atomic<bool> is_alive(true);
  std::thread optimizer(&MockTestPlant::runControlLoop, plant.get(), &is_alive);

  std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> loop_duration = std::chrono::steady_clock::now() - loop_start;
  // counter is number of dts
  for (int counter = 0; loop_duration.count() < test_duration * 1e3; counter++)
  {
    // wait until the correct hz has passed to tick the time
    // state at 100 Hz
    while (loop_duration.count() < (test_duration / 100) * 1e3 * counter)
    {
      usleep(50);
      loop_duration = std::chrono::steady_clock::now() - loop_start;
    }
    if (counter / 5 > iterations / 2)
    {  // this forces it to block
      plant->incrementTime(0.01);
      plant->updateState(state, plant->getPoseTime());
    }
  }
  is_alive.store(false);
  optimizer.join();

  // check all the things
  EXPECT_EQ(plant->checkStatus(), 1);
  EXPECT_EQ(plant->getStateTraj(), state_seq);
  EXPECT_EQ(plant->getControlTraj(), control_seq);
  // EXPECT_EQ(plant->getFeedbackState(), feedback);

  // check last pose update
  EXPECT_NE(plant->getLastUsedPoseUpdateTime(), 0.0);
  EXPECT_EQ(plant->getNumIter(), iterations / 2);
  EXPECT_EQ(plant->getLastOptimizationStride(), 1);

  double wait_ms = wait_s * 1e3;
  EXPECT_THAT(plant->getOptimizationDuration(),
              testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getOptimizationAvg(), testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getOptimizeLoopDuration(),
              testing::AllOf(testing::Ge(wait_ms * 2), testing::Le(wait_ms * 2 + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getLoopAvg(), testing::AllOf(testing::Ge(wait_ms * 2), testing::Le(wait_ms * 2 + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getFeedbackDuration(), testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getFeedbackAvg(), testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  // 10 iters of just waiting, 10 iters of waiting for correct time
  double expected_avg_wait = ((wait_ms * 3 * 10) + wait_ms * 10) / 10;
  EXPECT_THAT(plant->getSleepTimeAvg(),
              testing::AllOf(testing::Gt(expected_avg_wait), testing::Le(expected_avg_wait + SMALL_TIME_MS * 4)));
}

TEST_F(BasePlantTest, runControlLoopSlowed)
{
  mockController->initFeedback();

  int hz = plant->getHz();
  double test_duration = 1.0;  // in seconds for how long to run the test

  MockTestPlant::COST_PARAMS_T cost_params;
  MockTestPlant::DYN_PARAMS_T dyn_params;

  double wait_s = (1.0 / hz);

  auto wait_function = [wait_s](const Eigen::Ref<const MockController::state_array>& state,
                                int optimization_stride = 0) { usleep(wait_s * 1e6); };

  int iterations = int(std::round((hz * 1.0) / (test_duration)));  // number of times the method will be called

  // setup mock expected calls
  EXPECT_CALL(mockCost, setParams(testing::_)).Times(testing::AtLeast(54));  // 50 from waiting, 5 from run control
                                                                             // iteration
  EXPECT_CALL(mockDynamics, setParams(testing::_)).Times(testing::AtLeast(54));
  EXPECT_CALL(*mockController, resetControls()).Times(1);

  int expected_iters = iterations / 4 + 1;  // half do not count because of waiting, other half since we are slowed,
                                            // plus one for init call
  MockDynamics::control_array control = MockDynamics::control_array::Zero();
  EXPECT_CALL(*mockController, getCurrentControl(testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(46)
      .WillRepeatedly(testing::Return(control));  // called by update state should not be called since time since last
  // slide control sequence is skipped on the first iteration
  EXPECT_CALL(*mockController, slideControlSequence(1)).Times(1);
  EXPECT_CALL(*mockController, slideControlSequence(2)).Times(expected_iters - 2);
  EXPECT_CALL(*mockController, computeControl(testing::_, testing::_))
      .Times(expected_iters)
      .WillRepeatedly(testing::Invoke(wait_function));
  MockController::control_trajectory control_seq = MockController::control_trajectory::Zero();
  EXPECT_CALL(*mockController, getControlSeq()).Times(expected_iters).WillRepeatedly(testing::Return(control_seq));
  MockController::state_trajectory state_seq = MockController::state_trajectory::Zero();
  EXPECT_CALL(*mockController, getTargetStateSeq()).Times(expected_iters).WillRepeatedly(testing::Return(state_seq));
  EXPECT_CALL(*mockController, computeFeedback(testing::_))
      .Times(expected_iters)
      .WillRepeatedly(testing::Invoke(wait_function));
  MockController::TEMPLATED_FEEDBACK_STATE feedback;
  EXPECT_CALL(*mockController, getFeedbackState()).Times(expected_iters).WillRepeatedly(testing::Return(feedback));
  EXPECT_CALL(*mockController, computeFeedbackPropagatedStateSeq()).Times(expected_iters);
  EXPECT_CALL(*mockController, calculateSampledStateTrajectories()).Times(0);

  int init_time = 78;
  plant->setLastTime(init_time);
  MockDynamics::state_array state = MockDynamics::state_array::Zero();
  plant->updateState(state, init_time);

  std::atomic<bool> is_alive(true);
  std::thread optimizer(&MockTestPlant::runControlLoop, plant.get(), &is_alive);

  std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> loop_duration = std::chrono::steady_clock::now() - loop_start;
  // counter is number of dts
  for (int counter = 0; loop_duration.count() < test_duration * 1e3; counter++)
  {
    // wait until the correct hz has passed to tick the time
    // state at 100 Hz
    while (loop_duration.count() < (test_duration / 100) * 1e3 * counter)
    {
      usleep(50);
      loop_duration = std::chrono::steady_clock::now() - loop_start;
    }
    if (counter / 5 > iterations / 2)
    {  // this forces it to block
      plant->incrementTime(0.01);
      plant->updateState(state, plant->getPoseTime());
    }

    plant->setCostParams(cost_params);
    plant->setDynamicsParams(dyn_params);
  }
  is_alive.store(false);
  optimizer.join();

  // check all the things
  EXPECT_EQ(plant->checkStatus(), 1);
  EXPECT_EQ(plant->getStateTraj(), state_seq);
  EXPECT_EQ(plant->getControlTraj(), control_seq);
  // EXPECT_EQ(plant->getFeedbackState(), feedback);

  // check last pose update
  EXPECT_NE(plant->getLastUsedPoseUpdateTime(), 0.0);
  EXPECT_EQ(plant->getNumIter(), expected_iters);
  EXPECT_EQ(plant->getLastOptimizationStride(), 2);

  double wait_ms = wait_s * 1e3;
  EXPECT_THAT(plant->getOptimizationDuration(),
              testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getOptimizationAvg(), testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getOptimizeLoopDuration(),
              testing::AllOf(testing::Ge(wait_ms * 2), testing::Le(wait_ms * 2 + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getLoopAvg(), testing::AllOf(testing::Ge(wait_ms * 2), testing::Le(wait_ms * 2 + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getFeedbackDuration(), testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  EXPECT_THAT(plant->getFeedbackAvg(), testing::AllOf(testing::Ge(wait_ms), testing::Le(wait_ms + SMALL_TIME_MS)));
  // 10 iters of just waiting
  double expected_avg_wait = ((wait_ms * 2 * 10)) / 6;
  // EXPECT_THAT(plant->getSleepTimeAvg(), testing::AllOf(testing::Gt(expected_avg_wait - SMALL_TIME_MS),
  //                                                     testing::Le(expected_avg_wait + SMALL_TIME_MS * 4)));
}
