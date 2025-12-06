// ROS2 node that mirrors examples/spline_sub_bicycle_model_spatial_with_body_points.py
// Replace heavy dependencies with lighter approach: include ROS2 headers only if available at build
// time
#include "acados_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/srv/spline_debug.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cassert>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <thread>

using namespace std::chrono_literals;

class SplineInterfaceNode : public rclcpp::Node
{
public:
  using SplineDebug = autoware_internal_debug_msgs::srv::SplineDebug;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;

  SplineInterfaceNode() : Node("spline_interface")
  {
    RCLCPP_INFO(this->get_logger(), "spline_interface node started");

    // subscriptions (store latest messages)
    steering_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/"
      "optimised_steering",
      10, std::bind(&SplineInterfaceNode::steeringCallback, this, std::placeholders::_1));

    mpt_traj_sub_ = this->create_subscription<Trajectory>(
      "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/mpt_traj", 10,
      std::bind(&SplineInterfaceNode::mptTrajectoryCallback, this, std::placeholders::_1));

    service_ = this->create_service<SplineDebug>(
      "/acados_mpt_solver/get_optimised_trajectory",
      std::bind(
        &SplineInterfaceNode::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void steeringCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_steering_ = *msg;
  }

  void mptTrajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_mpt_traj_ = *msg;
  }

  void serviceCallback(
    const std::shared_ptr<SplineDebug::Request> req, std::shared_ptr<SplineDebug::Response> resp)
  {
    // Modularized: build parameters and x0, set parameters on solver, then call solver
    std::vector<double> x0;
    bool skipSolve = false;
    std::array<double, NP> parameters = buildParameters(req, x0, skipSolve, resp);
    if (skipSolve) return;

    {
      std::string x0_str = "x0: ";
      for (size_t i = 0; i < x0.size(); ++i) {
        if (i) x0_str += ", ";
        x0_str += std::to_string(x0[i]);
      }
      RCLCPP_INFO(this->get_logger(), "%s", x0_str.c_str());
    }

    setParametersToSolver(parameters);

    // warm-start and solve
    std::array<double, NX> x_init;
    std::array<double, NU> u_init;
    // use the built x0 as warm-start if available: x0 vector length may differ; we only copy first
    // NX entries
    if (!x0.empty()) {
      for (size_t i = 0; i < std::min((size_t)NX, x0.size()); ++i) x_init[i] = x0[i];
    }

    u_init[0] = 0.0;

    {
      std::string x_init_str = "x_init: ";
      for (size_t i = 0; i < x_init.size(); ++i) {
        if (i) x_init_str += ", ";
        x_init_str += std::to_string(x_init[i]);
      }
      RCLCPP_INFO(this->get_logger(), "%s", x_init_str.c_str());
    }

    // Do not warm-start the full horizon (Python only sets lbx/ubx at stage 0)
    auto solution = mpc_.getControl(x_init);
    RCLCPP_INFO(
      this->get_logger(), "Acados solve status: %s",
      solution.status == ACADOS_SUCCESS ? "Success" : "Failure");
    RCLCPP_INFO(this->get_logger(), "%s", solution.info.c_str());

    RCLCPP_INFO(this->get_logger(), "Retrieving results...");
    // Retrieve control trajectory and state trajectory (returned by value)
    auto utraj = solution.utraj;
    auto xtraj = solution.xtraj;

    // Fill response: optimised_steering = flattened utraj
    resp->optimised_steering.data.clear();
    for (size_t i = 0; i < N; ++i) {
      for (size_t j = 0; j < NU; ++j) {
        resp->optimised_steering.data.push_back((float)utraj[i][j]);
      }
    }

    // For trajectory, reconstruct simple points from state trajectory: eY and epsi combined with
    // reference
    size_t simN = N;  // number of points
    resp->optimised_trajectory.points.clear();
    for (size_t i = 0; i < simN; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint pt;
      double eY = xtraj[i][0];
      double epsi = xtraj[i][1];
      // double s_body_point_0 = xtraj[i][2]; // first body point's s value
      // double s_body_point_1 = xtraj[i][3]; // second body point's s value
      // double s_body_point_2 = xtraj[i][4]; // third body point's s value
      // double s_body_point_3 = xtraj[i][5]; // fourth body point's s value
      // double s_body_point_4 = xtraj[i][6]; // fifth body point's s value
      // double s_body_point_5 = xtraj[i][7]; // sixth body point's s value

      // RCLCPP_INFO(this->get_logger(), "s_body_point_0[%zu]=%.4f, s_body_point_1=%.4f,
      // s_body_point_2=%.4f, s_body_point_3=%.4f, s_body_point_4=%.4f, s_body_point_5=%.4f",
      //             i, s_body_point_0, s_body_point_1, s_body_point_2, s_body_point_3,
      //             s_body_point_4, s_body_point_5);
      pt.pose.position.x = eY;
      pt.pose.position.y = epsi;
      resp->optimised_trajectory.points.push_back(pt);
    }

    std::string optimised_steering_str = "";
    for (size_t i = 0; i < resp->optimised_steering.data.size(); ++i) {
      optimised_steering_str += std::to_string(resp->optimised_steering.data[i]) + ", ";
    }

    RCLCPP_INFO(this->get_logger(), "Optimized Steering: %s", optimised_steering_str.c_str());

    // std::string optimised_trajectory_str = "";
    // for (size_t i = 0; i < resp->optimised_trajectory.points.size(); ++i) {
    //     optimised_trajectory_str += "(" +
    //     std::to_string(resp->optimised_trajectory.points[i].pose.position.x) + ", " +
    //                                 std::to_string(resp->optimised_trajectory.points[i].pose.position.y)
    //                                 + "), ";
    // }

    // RCLCPP_INFO(this->get_logger(), "Optimized Trajectory: %s",
    // optimised_trajectory_str.c_str());
  }

  // Build parameter vector and initial state x0 from the request. If a parameter-size mismatch
  // is detected, this will set skipSolve=true and populate resp with empty results.
  std::array<double, NP> buildParameters(
    const std::shared_ptr<SplineDebug::Request> req, std::vector<double> & x0, bool & skipSolve,
    std::shared_ptr<SplineDebug::Response> resp)
  {
    skipSolve = false;
    // knots
    std::vector<double> knots(req->knots.data.begin(), req->knots.data.end());
    int n_segments = (int)knots.size() - 1;
    RCLCPP_ERROR(this->get_logger(), "Received request with %d segments", n_segments);

    // x_coeffs and y_coeffs are flattened arrays of length 4 * n_segments
    std::vector<double> x_coeffs_flat(req->x_coeffs.data.begin(), req->x_coeffs.data.end());
    std::vector<double> y_coeffs_flat(req->y_coeffs.data.begin(), req->y_coeffs.data.end());

    // curvatures
    std::vector<double> curvatures(req->curvatures.data.begin(), req->curvatures.data.end());

    int target_segments =
      CURVILINEAR_BICYCLE_MODEL_SPATIAL_N;  // number of segments (solver horizon)

    // Adjust sizes if necessary (simple strategy: extend last values)
    if (n_segments < target_segments) {
      RCLCPP_ERROR(
        this->get_logger(), "Extending from %d to %d segments", n_segments, target_segments);
      int n_missing = target_segments - n_segments;
      double last_knot = knots.back();
      double ds = 0.0;
      if (knots.size() >= 2) ds = (knots.back() - knots.front()) / (knots.size() - 1);
      for (int i = 0; i < n_missing; ++i) knots.push_back(last_knot + (i + 1) * ds);

      // extend coeffs by repeating last column
      for (int i = 0; i < 4; ++i) {
        double v = x_coeffs_flat[(n_segments - 1) * 4 + i];
        for (int j = 0; j < n_missing; ++j) x_coeffs_flat.push_back(v);
        v = y_coeffs_flat[(n_segments - 1) * 4 + i];
        for (int j = 0; j < n_missing; ++j) y_coeffs_flat.push_back(v);
      }

      // extend curvatures
      if (!curvatures.empty()) {
        double last_k = curvatures.back();
        for (int i = 0; i < n_missing; ++i) curvatures.push_back(last_k);
      }
    } else if (n_segments > target_segments) {
      RCLCPP_ERROR(
        this->get_logger(), "Clipping from %d to %d segments", n_segments, target_segments);
      // clip to exactly target_segments segments
      knots.resize(target_segments);
      x_coeffs_flat.resize(4 * (target_segments - 1));
      y_coeffs_flat.resize(4 * (target_segments - 1));
      curvatures.resize(4 * (target_segments - 1));
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "sizes: knots=%zu x_coeffs=%zu y_coeffs=%zu curvatures=%zu body_points=%zu", knots.size(),
      x_coeffs_flat.size(), y_coeffs_flat.size(), curvatures.size(), req->body_points.size());

    if (req->body_points.size() != req->body_points_curvilinear.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "body points length mismatch: body_points=%zu body_points_curvilinear=%zu",
        req->body_points.size(), req->body_points_curvilinear.size());
      assert(
        req->body_points.size() == req->body_points_curvilinear.size() && "body points mismatch");
    }

    // body points curvilinear -> vector of doubles (s values then eY values)
    std::vector<double> body_points_curvilinear;
    for (const auto & pt : req->body_points_curvilinear) body_points_curvilinear.push_back(pt.x);
    for (const auto & pt : req->body_points_curvilinear) body_points_curvilinear.push_back(pt.y);

    // body points global
    std::vector<double> body_points_xy;
    for (const auto & pt : req->body_points) body_points_xy.push_back(pt.x);
    for (const auto & pt : req->body_points) body_points_xy.push_back(pt.y);

    // Build parameters vector similar to Python
    std::array<double, NP> parameters;
    double s_interp = 0.0;
    size_t idx = 0;

    // 1. s_interp
    parameters[idx++] = s_interp;

    // 2. knots
    for (double v : knots) {
      parameters[idx++] = v;
    }

    // 3. x_coeffs_flat
    for (double v : x_coeffs_flat) {
      parameters[idx++] = v;
    }

    // 4. knots again
    for (double v : knots) {
      parameters[idx++] = v;
    }

    // 5. y_coeffs_flat
    for (double v : y_coeffs_flat) {
      parameters[idx++] = v;
    }

    // 6. knots again
    for (double v : knots) {
      parameters[idx++] = v;
    }

    // 7. curvatures
    for (double v : curvatures) {
      parameters[idx++] = v;
    }

    // 8. body_points_xy
    for (double v : body_points_xy) {
      parameters[idx++] = v;
    }

    parameters[idx++] = lf;
    parameters[idx++] = lr;

    // set x0: initial state vector
    x0.clear();
    x0.push_back(0.0);
    x0.push_back(0.0);
    x0.insert(x0.end(), body_points_curvilinear.begin(), body_points_curvilinear.end());

    // store reference path length for per-stage s_interp construction
    if (!knots.empty()) sref_ = knots.back();

    // sanity-check NP
    if (CURVILINEAR_BICYCLE_MODEL_SPATIAL_NP > 0) {
      size_t actual_np = parameters.size();
      size_t expected_np = (size_t)CURVILINEAR_BICYCLE_MODEL_SPATIAL_NP;
      if (actual_np != expected_np) {
        size_t sample = std::min((size_t)10, actual_np);
        std::string s_first = "";
        for (size_t i = 0; i < sample; ++i) s_first += std::to_string(parameters[i]) + ", ";
        std::string s_last = "";
        for (size_t i = (actual_np > sample ? actual_np - sample : 0); i < actual_np; ++i)
          s_last += std::to_string(parameters[i]) + ", ";
        RCLCPP_ERROR(
          this->get_logger(),
          "parameter length mismatch: actual=%zu expected=%zu; first=%s last=%s", actual_np,
          expected_np, s_first.c_str(), s_last.c_str());
        resp->optimised_steering.data.clear();
        resp->optimised_trajectory.points.clear();
        RCLCPP_ERROR(this->get_logger(), "Skipping solve due to parameter-size mismatch");
        skipSolve = true;
      }
    }

    return parameters;
  }

  // Set parameters on the AcadosInterface for all stages. We update the first parameter (s_interp)
  // per stage to match the Python behavior that calls `set(j, "p", parameters)` in a loop.
  void setParametersToSolver(const std::array<double, NP> & parameters)
  {
    for (int stage = 0; stage < (int)N; ++stage) {
      std::array<double, NP> params_copy = parameters;
      double s_interp = 0.0;
      if (sref_ > 0.0) {
        s_interp = sref_ * ((double)stage / (double)N);
      }
      params_copy[0] = s_interp;
      mpc_.setParameters(stage, params_copy);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr steering_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr mpt_traj_sub_;
  rclcpp::Service<autoware_internal_debug_msgs::srv::SplineDebug>::SharedPtr service_;

  std::mutex mutex_;
  std_msgs::msg::Float32MultiArray latest_steering_;
  autoware_planning_msgs::msg::Trajectory latest_mpt_traj_;
  double lf{4.89};
  double lr{0.0};
  // reference path length used to compute per-stage s_interp
  double sref_{0.0};

  AcadosInterface mpc_{20, 1e-6};  // set max_iter and tol at construction
};

int main(int argc, char ** argv)
{
  // Disable buffering on stdout/stderr so prints appear immediately
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SplineInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
