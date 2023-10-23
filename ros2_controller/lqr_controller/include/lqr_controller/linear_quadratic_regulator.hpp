#ifndef ROS2_CONTROLLER__LQR_CONTROLLER__LINEAR_QUADRATIC_REGULATOR_HPP_
#define ROS2_CONTROLLER__LQR_CONTROLLER__LINEAR_QUADRATIC_REGULATOR_HPP_

#include "Eigen/Core"

namespace ros2_controller {
namespace lqr_controller {

/**
 * @brief Solver for discrete-time linear quadratic problem.
 * @param A The system dynamic matrix
 * @param B The control matrix
 * @param Q The cost matrix for system state
 * @param R The cost matrix for control output
 * @param M is the cross term between x and u, i.e. x'Qx + u'Ru + 2x'Mu
 * @param tolerance The numerical tolerance for solving Discrete
 *        Algebraic Riccati equation (DARE)
 * @param max_num_iteration The maximum iterations for solving ARE
 * @param ptr_K The feedback control matrix (pointer)
 */
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &M, const double tolerance,
                     const uint max_num_iteration, Eigen::MatrixXd *ptr_K);

/**
 * @brief Solver for discrete-time linear quadratic problem.
 * @param A The system dynamic matrix
 * @param B The control matrix
 * @param Q The cost matrix for system state
 * @param R The cost matrix for control output
 * @param tolerance The numerical tolerance for solving Discrete
 *        Algebraic Riccati equation (DARE)
 * @param max_num_iteration The maximum iterations for solving ARE
 * @param ptr_K The feedback control matrix (pointer)
 */
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const double tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K);

}  // namespace lqr_controller
}  // namespace ros2_controller

#endif  // ROS2_CONTROLLER__LQR_CONTROLLER__LINEAR_QUADRATIC_REGULATOR_HPP_