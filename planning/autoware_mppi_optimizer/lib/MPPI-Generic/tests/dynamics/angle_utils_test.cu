#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <mppi/utils/angle_utils.cuh>
#include <random>

TEST(AngleUtils, normalizeAngleKnownDouble)
{
  double angle = 0.5;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), 0.5);

  angle = M_PI_2;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), M_PI_2);

  angle = M_PI;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), M_PI);

  angle = -M_PI;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), M_PI);

  angle = -M_PI * 3;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), M_PI);

  angle = M_PI * 8;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), 0);
}

TEST(AngleUtils, normalizeAngleKnownFloat)
{
  float angle = 0.5f;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), 0.5f);

  angle = static_cast<float>(M_PI_2);
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), static_cast<float>(M_PI_2));

  angle = static_cast<float>(M_PI);
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), static_cast<float>(M_PI));

  angle = -static_cast<float>(M_PI);
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), static_cast<float>(M_PI));

  angle = -static_cast<float>(M_PI) * 3.0f + 1e-3f;
  EXPECT_FLOAT_EQ(angle_utils::normalizeAngle(angle), -static_cast<float>(M_PI) + 1e-3f);

  angle = static_cast<float>(M_PI) * 8.0f;
  EXPECT_NEAR(angle_utils::normalizeAngle(angle), 0.0f, 1e-6);
}

TEST(AngleUtils, normalizeAngleRandom)
{
  std::random_device rd;
  std::mt19937 e2(rd());
  std::uniform_real_distribution<> dist(-100, 100);

  for (int i = 0; i < 1000; i++)
  {
    double rand = dist(e2);
    double result = angle_utils::normalizeAngle(rand);
    EXPECT_TRUE(result <= M_PI && result > -M_PI) << result;
  }

  for (int i = 0; i < 1000; i++)
  {
    float rand = dist(e2);
    float result = angle_utils::normalizeAngle(rand);
    EXPECT_TRUE(result <= M_PI && result > -M_PI) << result;
  }
}

TEST(AngleUtils, shortestAngularDistanceDouble)
{
  double angle_1 = 0;
  double angle_2 = 0;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), 0);

  angle_1 = 0;
  angle_2 = M_PI_2;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI_2);

  angle_1 = 0;
  angle_2 = -M_PI;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI);

  angle_1 = M_PI_4;
  angle_2 = -M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), -M_PI_2);

  angle_1 = -M_PI_4;
  angle_2 = M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI_2);

  angle_1 = 3 * M_PI_4;
  angle_2 = -3 * M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI_2);

  angle_1 = -3 * M_PI_4;
  angle_2 = 3 * M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), -M_PI_2);
}

TEST(AngleUtils, shortestAngularDistanceFloat)
{
  float angle_1 = 0;
  float angle_2 = 0;
  EXPECT_NEAR(angle_utils::shortestAngularDistance(angle_1, angle_2), 0, 1e-6);

  angle_1 = 0;
  angle_2 = M_PI_2;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI_2);

  // TODO is right?
  angle_1 = 0;
  angle_2 = -M_PI;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI);

  angle_1 = M_PI_4;
  angle_2 = -M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), -M_PI_2);

  angle_1 = -M_PI_4;
  angle_2 = M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI_2);

  angle_1 = 3 * M_PI_4;
  angle_2 = -3 * M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), M_PI_2);

  angle_1 = -3 * M_PI_4;
  angle_2 = 3 * M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::shortestAngularDistance(angle_1, angle_2), -M_PI_2);
}

TEST(AngleUtils, interpolateEulerAngleLinearDouble)
{
  double angle_1 = 0;
  double angle_2 = 0;
  double alpha = 0.5;
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, alpha), 0);

  angle_1 = 0;
  angle_2 = M_PI_2;
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.0), 0);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.25), M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.5), M_PI_4);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.75), M_PI_4 * 3 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 1.0), M_PI_2);

  angle_1 = M_PI_4;
  angle_2 = -M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.0), M_PI_4);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.25), M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.5), 0);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.75), -M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 1.0), -M_PI_4);

  angle_1 = 3 * M_PI_4;
  angle_2 = -3 * M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.0), 3 * M_PI_4);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.25), 7 * M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.5), M_PI);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.75), -7 * M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 1.0), -3 * M_PI_4);
}

TEST(AngleUtils, interpolateEulerAngleLinearFloat)
{
  float angle_1 = 0;
  float angle_2 = 0;
  float alpha = 0.5;
  EXPECT_NEAR(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, alpha), 0, 1e-7);

  angle_1 = 0;
  angle_2 = M_PI_2;
  EXPECT_NEAR(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.0f), 0, 1e-7);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.25f), M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.5f), M_PI_4);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.75f), M_PI_4 * 3 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 1.0f), M_PI_2);

  angle_1 = M_PI_4;
  angle_2 = -M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.0f), M_PI_4);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.25f), M_PI_4 / 2);
  EXPECT_NEAR(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.5f), 0, 1e-7);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.75f), -M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 1.0f), -M_PI_4);

  angle_1 = 3 * M_PI_4;
  angle_2 = -3 * M_PI_4;
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.0f), 3 * M_PI_4);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.25f), 7 * M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.5f), M_PI);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 0.75f), -7 * M_PI_4 / 2);
  EXPECT_FLOAT_EQ(angle_utils::interpolateEulerAngleLinear(angle_1, angle_2, 1.0f), -3 * M_PI_4);
}
