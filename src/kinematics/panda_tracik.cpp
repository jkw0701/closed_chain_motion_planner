#include <closed_chain_motion_planner/kinematics/panda_tracik.h>


Eigen::Matrix3d getEigenRotation(const KDL::Rotation & r)
{
  Eigen::Matrix3d matrix;
  for (int i = 0 ;i < 9; i++)
  {
     matrix(i/3, i%3) = r.data[i];
  }
  return matrix;
}

Eigen::Vector3d getEigenVector(const KDL::Vector& v)
{
  Eigen::Vector3d vector;
  for (int i=0; i<3; i++)
  {
    vector(i) = v(i);
  }
  return vector;
}

Eigen::Isometry3d getEigenFrame(const KDL::Frame & frame)
{
  Eigen::Isometry3d transform;
  transform.translation() = getEigenVector(frame.p);
  transform.linear() = getEigenRotation(frame.M);
  return transform;
}

KDL::Rotation getKDLRotation(const Eigen::Matrix3d & matrix)
{
  KDL::Rotation r;
  for (int i=0 ;i<9; i++)
  {
    r.data[i] = matrix(i/3, i%3);
  }
  return r;
}

KDL::Vector getKDLVector(const Eigen::Vector3d& vector)
{
  KDL::Vector v;
  for (int i=0; i<3; i++)
  {
    v(i) = vector(i);
  }
  return v;
}


KDL::Frame getKDLFrame(const Eigen::Isometry3d & transform)
{
  KDL::Frame frame;
  frame.p = getKDLVector(transform.translation());
  frame.M = getKDLRotation(transform.linear());
  return frame;
}


Eigen::VectorXd TrackIKAdaptor::getRandomConfig()
{
  double stdDev = 0.3;
  Eigen::VectorXd random_;
  random_.resize(nominal_.data.size());
  for (unsigned int i = 0; i < nominal_.data.size(); i++)
  {
    // stdDev = (ub_[i] - lb_[i]) / 4;
    // stdDev = 0.2;
    double v = gaussian(nominal_.data[i], stdDev);
    if (v < lb_[i]) v = lb_[i];
    else if (v > ub_[i]) v = ub_[i];
    random_[i] = v;
  }
  return random_;
  // return length.asDiagonal() * Eigen::VectorXd::Random(7) + length + lb_;
}

bool TrackIKAdaptor::solve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution)
{
  return solve(nominal_.data, transform, solution);
}

bool TrackIKAdaptor::randomSolve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution)
{  
  return solve(getRandomConfig(), transform, solution);
}

double TrackIKAdaptor::gaussian(double mean, double stddev)
{
  return normalDist_(generator_) * stddev + mean;
}


bool TrackIKAdaptor::isValid(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  double eps =  0.001;
  for (int i=0; i<lb_.size(); i++)
  {
    if (q(i) < lb_(i) + eps) return false;
    if (q(i) > ub_(i) - eps) return false;
  }
  return true;
}




panda_ik::panda_ik(const std::string & arm_name) : tracik_solver_(arm_name + "_link0", arm_name + "_hand")
{
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver_.getKDLChain(chain);
  if (!valid){ROS_ERROR("There was no valid KDL chain found");return;}
  valid = tracik_solver_.getKDLLimits(ll, ul);
  if (!valid){ std::cout << "error~~~~~`" << std::endl; return;}

  n_joint_ = chain.getNrOfJoints();
  assert(n_joint_ == ll.data.size());
  assert(n_joint_ == ul.data.size());

  lb_ = ll.data;
  ub_ = ul.data;
  nominal_.resize(chain.getNrOfJoints());

  for (uint j = 0; j < nominal_.data.size(); j++)
  {
    nominal_(j) = (ll(j) + ul(j)) / 2.0;
  }

  fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
}

void panda_ik::SetSolveType(TRAC_IK::SolveType type)
{
  tracik_solver_.SetSolveType(type);
}

bool panda_ik::solve(const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution)
{
  KDL::Frame target_frame;
  KDL::JntArray jarr_q0;
  KDL::JntArray result_q;
  result_q.resize(n_joint_);
  jarr_q0.data = q0;

  target_frame = getKDLFrame(transform);
  mutex_.lock();
  int result = tracik_solver_.CartToJnt(jarr_q0, target_frame, result_q);
  mutex_.unlock();
  if (result >= 0)
  {
    solution = result_q.data;
    return true;
  }
  return false;
}


Eigen::Isometry3d panda_ik::fk(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  KDL::JntArray jarr_q;
  KDL::Frame frame;
  
  jarr_q.resize(7);
  jarr_q.data = q;
  fk_solver->JntToCart(jarr_q, frame);

  return getEigenFrame(frame);
}

Eigen::Isometry3d panda_ik::random_fk()
{
  return fk(getRandomConfig());
}

