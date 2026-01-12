#include <franka_gazebo/model_kdl.h>

#include <eigen_conversions/eigen_kdl.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/solveri.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace franka_gazebo {

int ModelKDL::segment(franka::Frame frame) {
  // clang-format off
  switch (frame) {
    case franka::Frame::kJoint1:      return 1;
    case franka::Frame::kJoint2:      return 2;
    case franka::Frame::kJoint3:      return 3;
    case franka::Frame::kJoint4:      return 4;
    case franka::Frame::kJoint5:      return 5;
    case franka::Frame::kJoint6:      return 6;
    case franka::Frame::kJoint7:      return 7;
    case franka::Frame::kFlange:      return 8;
    case franka::Frame::kEndEffector: return 9;
    case franka::Frame::kStiffness:   return 10;
    default: return -1;
  }
  // clang-format on
}

bool ModelKDL::isCloseToSingularity(const KDL::Jacobian& jacobian) const {
  if (this->singularity_threshold_ < 0) {
    return false;
  }
  Eigen::Matrix<double, 6, 6> mat = jacobian.data * jacobian.data.transpose();
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(mat);
  double critical = svd.singularValues().tail(1)(0);
  return critical < this->singularity_threshold_;
}

// Implementation copied from <kdl/isolveri.hpp> because
// KDL::ChainDynSolver inherits *privately* from SolverI ... -.-'
std::string ModelKDL::strError(const int error) {
  // clang-format off
  switch(error) {
  case KDL::SolverI::E_NOERROR:                 return "No error"; break;
  case KDL::SolverI::E_NO_CONVERGE:             return "Failed to converge"; break;
  case KDL::SolverI::E_UNDEFINED:               return "Undefined value"; break;
  case KDL::SolverI::E_DEGRADED:                return "Converged but degraded solution"; break;
#if ROS_VERSION_MINIMUM(1, 15, 0)
  // These were introduced in melodic
  case KDL::SolverI::E_NOT_UP_TO_DATE:          return "Internal data structures not up to date with Chain"; break;
  case KDL::SolverI::E_SIZE_MISMATCH:           return "The size of the input does not match the internal state"; break;
  case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED: return "The maximum number of iterations is exceeded"; break;
  case KDL::SolverI::E_OUT_OF_RANGE:            return "The requested index is out of range"; break;
  case KDL::SolverI::E_NOT_IMPLEMENTED:         return "The requested function is not yet implemented"; break;
  case KDL::SolverI::E_SVD_FAILED:              return "SVD failed"; break;
#endif
  default: return "UNKNOWN ERROR";
  }
  // clang-format on
}
ModelKDL::ModelKDL(const urdf::Model& model,
                   const std::string& root,
                   const std::string& tip,
                   double singularity_threshold)
    : singularity_threshold_(singularity_threshold) {
  KDL::Tree tree;
  if (not kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::invalid_argument("Cannot construct KDL tree from URDF");
  }

  if (not tree.getChain(root, tip, this->chain_)) {
    throw std::invalid_argument("Cannot find chain within URDF tree from root '" + root +
                                "' to tip '" + tip + "'. Do these links exist?");
  }

  ROS_INFO_STREAM("KDL Model initialized for chain from '" << root << "' -> '" << tip << "'");
}

void ModelKDL::augmentFrame(const std::string& name,
                            const std::array<double, 16>& transform,
                            KDL::Chain& chain) {
  Eigen::Affine3d t;
  KDL::Frame f;
  t.matrix() = Eigen::Matrix4d(transform.data());
  tf::transformEigenToKDL(t, f);
  chain.addSegment(KDL::Segment(name, KDL::Joint(KDL::Joint::None), f));
}

void ModelKDL::augmentFrame(const std::string& name,
                            const std::array<double, 3>& center_of_mass,
                            double mass,
                            const std::array<double, 9>& inertia,
                            KDL::Chain& chain) {
  KDL::Vector kc;
  KDL::RotationalInertia ki;
  std::copy(center_of_mass.begin(), center_of_mass.end(), std::begin(kc.data));
  std::copy(inertia.begin(), inertia.end(), std::begin(ki.data));

  chain.addSegment(KDL::Segment(name, KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector::Zero()),
                                KDL::RigidBodyInertia(mass, kc, ki)));
}

std::array<double, 16> ModelKDL::pose(
    franka::Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Frame kp;

  // Agument the chain with the two new Frames 'EE' and 'K'
  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("EE", F_T_EE, chain);
  augmentFrame("K", EE_T_K, chain);

  KDL::ChainFkSolverPos_recursive solver(chain);

  kq.data = Eigen::Matrix<double, 7, 1>::Map(q.data());

  int error = solver.JntToCart(kq, kp, segment(frame));
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL forward kinematics pose calculation failed with error: " +
                           strError(error));
  }
  Eigen::Affine3d p;
  tf::transformKDLToEigen(kp, p);

  std::array<double, 16> result;
  Eigen::MatrixXd::Map(&result[0], 4, 4) = p.matrix();

  return result;
}

std::array<double, 42> ModelKDL::bodyJacobian(
    franka::Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Jacobian J(7);  // NOLINT(readability-identifier-naming)
  kq.data = Eigen::Matrix<double, 7, 1>::Map(q.data());

  // Augment the chain with the two virtual frames 'EE' and 'K'
  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("EE", F_T_EE, chain);
  augmentFrame("K", EE_T_K, chain);

  KDL::ChainJntToJacSolver solver(chain);

  int seg = segment(frame);
  int error = solver.JntToJac(kq, J, seg);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL zero jacobian calculation failed with error: " + strError(error));
  }

  // Shift the zero jacobian to the "body" frame by using the rotation of the FK of that frame
  KDL::Frame f = KDL::Frame::Identity();
  auto transform = pose(frame, q, F_T_EE, EE_T_K);
  Eigen::Affine3d t;
  t.matrix() = Eigen::Matrix4d(transform.data());
  tf::transformEigenToKDL(t, f);

  J.changeBase(f.M.Inverse());

  // Singularity check
  if (isCloseToSingularity(J)) {
    ROS_WARN_STREAM_THROTTLE(1, "Body Jacobian close to singularity");
  }

  std::array<double, 42> result;
  Eigen::MatrixXd::Map(&result[0], 6, 7) = J.data;

  return result;
}

std::array<double, 42> ModelKDL::zeroJacobian(
    franka::Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Jacobian J(7);  // NOLINT(readability-identifier-naming)
  kq.data = Eigen::Matrix<double, 7, 1>::Map(q.data());

  // Augment the chain with the two virtual frames 'EE' and 'K'
  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("EE", F_T_EE, chain);
  augmentFrame("K", EE_T_K, chain);

  KDL::ChainJntToJacSolver solver(chain);

  int error = solver.JntToJac(kq, J, segment(frame));
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL zero jacobian calculation failed with error: " + strError(error));
  }

  // Singularity Check
  if (isCloseToSingularity(J)) {
    ROS_WARN_STREAM_THROTTLE(1, "Zero Jacobian close to singularity");
  }

  std::array<double, 42> result;
  Eigen::MatrixXd::Map(&result[0], 6, 7) = J.data;

  return result;
}

std::array<double, 49> ModelKDL::mass(
    const std::array<double, 7>& q,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::JntSpaceInertiaMatrix M(7);  // NOLINT(readability-identifier-naming)
  kq.data = Eigen::Matrix<double, 7, 1>::Map(q.data());

  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("load", F_x_Ctotal, m_total, I_total, chain);
  KDL::ChainDynParam solver(chain, KDL::Vector(0, 0, -9.81));

  int error = solver.JntToMass(kq, M);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL mass calculation failed with error: " + strError(error));
  }

  std::array<double, 49> result;
  Eigen::MatrixXd::Map(&result[0], 7, 7) = M.data;

  return result;
}

std::array<double, 7> ModelKDL::coriolis(
    const std::array<double, 7>& q,
    const std::array<double, 7>& dq,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq, kdq, kc(7);
  kq.data = Eigen::Matrix<double, 7, 1>::Map(q.data());
  kdq.data = Eigen::Matrix<double, 7, 1>::Map(dq.data());

  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("load", F_x_Ctotal, m_total, I_total, chain);
  KDL::ChainDynParam solver(chain, KDL::Vector(0, 0, -9.81));

  int error = solver.JntToCoriolis(kq, kdq, kc);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL coriolis calculation failed with error: " + strError(error));
  }

  std::array<double, 7> result;
  Eigen::VectorXd::Map(&result[0], kc.data.size()) = kc.data;

  return result;
}

std::array<double, 7> ModelKDL::gravity(
    const std::array<double, 7>& q,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
    const std::array<double, 3>& gravity_earth) const {
  KDL::JntArray kq, kg(7);
  KDL::Vector grav(gravity_earth[0], gravity_earth[1], gravity_earth[2]);
  kq.data = Eigen::Matrix<double, 7, 1>::Map(q.data());

  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("load", F_x_Ctotal, m_total, {1, 0, 0, 0, 1, 0, 0, 0, 1}, chain);
  KDL::ChainDynParam solver(chain, grav);

  int error = solver.JntToGravity(kq, kg);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL gravity calculation failed with error: " + strError(error));
  }

  std::array<double, 7> result;
  Eigen::VectorXd::Map(&result[0], kg.data.size()) = kg.data;

  return result;
}
}  // namespace franka_gazebo
