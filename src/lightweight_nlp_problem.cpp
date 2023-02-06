//
// Created by yenkn on 23-1-27.
//
#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>
#include <coin/IpTNLP.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "liom_local_planner/lightweight_nlp_problem.h"
#include "liom_local_planner/time.h"

#define tag_f 1
#define tag_g 2
#define tag_L 3

namespace liom_local_planner {

class LiomIPOPTInterface : public Ipopt::TNLP
{
public:
  LiomIPOPTInterface(double w_inf, const Constraints &profile, const FullStates &guess, const PlannerConfig &config)
    : w_inf_(w_inf), profile_(profile), guess_(guess), config_(config) {
    nfe_ = guess_.states.size();
    nrows_ = nfe_ - 2;
    disc_nvar_ = config_.vehicle.n_disc * 2;
    ncols_ = NVar + disc_nvar_;

    // tf + [x y theta v phi a omega x_disc_0 y_disc_0 ... x_disc_n y_disc_n] (nfe * (7 + n * 2)) + theta_end
    nvar_ = 1 + nrows_ * ncols_ + 1;
    result_.resize(nvar_);
    x0_.resize(nvar_);
  }

  virtual ~LiomIPOPTInterface() = default;

  LiomIPOPTInterface(const LiomIPOPTInterface&) = delete;
  LiomIPOPTInterface& operator=(const LiomIPOPTInterface&) = delete;

  virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = nvar_;
    m = 0;
    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);
    // use the C style indexing (0-based)
    index_style = C_STYLE;
    return true;
  }

  virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) {
    Eigen::Map<Eigen::ArrayXXd> lb_mat(x_l + 1, nrows_, ncols_);
    x_l[0] = 0.1; // tf > 0.1
    x_u[0] = inf;
    x_l[nvar_ - 1] = -inf;
    x_u[nvar_ - 1] = inf;

    TrajectoryPointVector lb_vec;
    lb_vec << -inf, -inf, -inf,
        config_.vehicle.min_velocity,
        -config_.vehicle.phi_max,
        -config_.vehicle.max_acceleration,
        -config_.vehicle.omega_max;
    lb_mat.leftCols<NVar>() = lb_vec.transpose().replicate(nrows_, 1);
    lb_mat.rightCols(disc_nvar_) = profile_.corridor_lb.middleRows(1, nrows_);

    Eigen::Map<Eigen::ArrayXXd> ub_mat(x_u + 1, nrows_, ncols_);

    TrajectoryPointVector ub_vec;
    ub_vec << inf, inf, inf,
        config_.vehicle.max_velocity,
        config_.vehicle.phi_max,
        config_.vehicle.max_acceleration,
        config_.vehicle.omega_max;
    ub_mat.leftCols<NVar>() = ub_vec.transpose().replicate(nrows_, 1);
    ub_mat.rightCols(disc_nvar_) = profile_.corridor_ub.middleRows(1, nrows_);
    return true;
  }

  virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda,
                                  Ipopt::Number* lambda) {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    Eigen::Map<Eigen::ArrayXXd> x0_mat(x + 1, nrows_, ncols_);
    auto x0_state_mat = x0_mat.leftCols<NVar>();
    auto x0_disc_mat = x0_mat.rightCols(disc_nvar_);
    x[0] = guess_.tf;
    x[nvar_ - 1] = profile_.goal.theta;

    for(int i = 0; i < nrows_; i++) {
      x0_state_mat.row(i) = guess_.states[i+1].vec();

      auto x0_disc = config_.vehicle.GetDiscPositions(guess_.states[i+1].x, guess_.states[i+1].y, guess_.states[i+1].theta);
      x0_disc_mat.row(i) = Eigen::Map<Eigen::VectorXd>(x0_disc.data(), x0_disc.size());
    }

    return true;
  }

  template<class T> T eval_infeasibility(const T *x) {
    T dt = x[0] / nfe_;

    T infeasibility = 0.0;
    infeasibility += pow(x[1 + (0) * nrows_ + 0] - profile_.start.x - dt * profile_.start.v * cos(profile_.start.theta), 2)
                     + pow(x[1 + (1) * nrows_ + 0] - profile_.start.y - dt * profile_.start.v * sin(profile_.start.theta), 2)
                     + pow(x[1 + (2) * nrows_ + 0] - profile_.start.theta - dt * profile_.start.v * tan(profile_.start.phi) / config_.vehicle.wheel_base, 2)
                     + pow(x[1 + (3) * nrows_ + 0] - profile_.start.v - dt * profile_.start.a, 2)
                     + pow(x[1 + (4) * nrows_ + 0] - profile_.start.phi - dt * profile_.start.omega, 2);

    for(int i = 1; i < nrows_; i++) {
      infeasibility += pow(x[1 + (0) * nrows_ + i] - x[1 + (0) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * cos(x[1 + (2) * nrows_ + i-1]), 2)
                       + pow(x[1 + (1) * nrows_ + i] - x[1 + (1) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * sin(x[1 + (2) * nrows_ + i-1]), 2)
                       + pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * tan(x[1 + (4) * nrows_ + i-1]) / config_.vehicle.wheel_base, 2)
                       + pow(x[1 + (3) * nrows_ + i] - x[1 + (3) * nrows_ + i-1] - dt * x[1 + (5) * nrows_ + i-1], 2)
                       + pow(x[1 + (4) * nrows_ + i] - x[1 + (4) * nrows_ + i-1] - dt * x[1 + (6) * nrows_ + i-1], 2);
    }

    infeasibility += pow(profile_.goal.x - x[1 + (0) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * cos(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(profile_.goal.y - x[1 + (1) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * sin(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(x[nvar_ - 1] - x[1 + (2) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * tan(x[1 + (4) * nrows_ + nrows_-1]) / config_.vehicle.wheel_base, 2)
                     + pow(profile_.goal.v - x[1 + (3) * nrows_ + nrows_-1] - dt * x[1 + (5) * nrows_ + nrows_-1], 2)
                     + pow(profile_.goal.phi - x[1 + (4) * nrows_ + nrows_-1] - dt * x[1 + (6) * nrows_ + nrows_-1], 2);

    infeasibility += pow(sin(profile_.goal.theta) - sin(x[nvar_ - 1]), 2) + pow(cos(profile_.goal.theta) - cos(x[nvar_ - 1]), 2);

    for(int i = 0; i < nrows_; i++) {
      for (int j = 0; j < config_.vehicle.n_disc; j++) {
        infeasibility += pow(x[1 + (NVar + j * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * cos(x[1 + (2) * nrows_ + i]), 2)
                         + pow(x[1 + (NVar + j * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * sin(x[1 + (2) * nrows_ + i]), 2);
      }
    }

    return infeasibility;
  }

  template<class T> bool eval_obj(Ipopt::Index n, const T *x, T& obj_value) {
    obj_value = x[0];

    for(int i = 0; i < nrows_; i++) {
      obj_value += config_.opti_w_a * x[1 + 5 * nrows_ + i] * x[1 + 5 * nrows_ + i]
          + config_.opti_w_omega * x[1 + 6 * nrows_ + i] * x[1 + 6 * nrows_ + i];
    }

    obj_value += w_inf_ * eval_infeasibility(x);
    return true;
  }

  template<class T> bool eval_constraints(Ipopt::Index n, const T *x, Ipopt::Index m, T *g) {
    return true;
  }

  virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) {
    eval_obj(n, x, obj_value);
    return true;
  }

  virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) {
    gradient(tag_f, n, x, grad_f);
    return true;
  }

  virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) {
    eval_constraints(n, x, m, g);
    return true;
  }

  virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                          Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure of the jacobian
      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
      }
    }
    return true;
  }

  virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                      Ipopt::Index* jCol, Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure. This is a symmetric matrix, fill the lower left triangle only.
      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        iRow[idx] = rind_L[idx];
        jCol[idx] = cind_L[idx];
      }
    } else {
      // return the values. This is a symmetric matrix, fill the lower left triangle only
      obj_lam[0] = obj_factor;
      for(Ipopt::Index idx = 0; idx < m; idx++)
        obj_lam[1 + idx] = lambda[idx];

      set_param_vec(tag_L, m + 1, obj_lam);
      sparse_hess(tag_L, n, 1, const_cast<double *>(x), &nnz_L, &rind_L, &cind_L, &hessval, options_L);

      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        values[idx] = hessval[idx];
      }
    }

    return true;
  }

  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                 Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                 Ipopt::Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq) {
    std::copy(x, x + n, result_.data());

    delete[] obj_lam;
    free(rind_g);
    free(cind_g);
    free(rind_L);
    free(cind_L);
    free(jacval);
    free(hessval);
  }


  /** Method to generate the required tapes */
  virtual void generate_tapes(Ipopt::Index n, Ipopt::Index m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag) {
    Ipopt::Number *lamp  = new double[m];
    Ipopt::Number *zl    = new double[m];
    Ipopt::Number *zu    = new double[m];

    adouble *xa   = new adouble[n];
    adouble *g    = new adouble[m];
    double *lam   = new double[m];
    double sig;
    adouble obj_value;

    double dummy;
    obj_lam   = new double[m+1];
    get_starting_point(n, true, x0_.data(), false, zl, zu, m, false, lamp);

    trace_on(tag_f);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

    trace_off();

    trace_on(tag_g);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_constraints(n,xa,m,g);


    for(Ipopt::Index idx=0;idx<m;idx++)
      g[idx] >>= dummy;

    trace_off();

    trace_on(tag_L);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];
    for(Ipopt::Index idx=0;idx<m;idx++)
      lam[idx] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);

    for(Ipopt::Index idx=0;idx<m;idx++)
      obj_value += g[idx]*mkparam(lam[idx]);

    obj_value >>= dummy;

    trace_off();

    rind_g = nullptr;
    cind_g = nullptr;
    rind_L = nullptr;
    cind_L = nullptr;

    options_g[0] = 0;          /* sparsity pattern by index domains (default) */
    options_g[1] = 0;          /*                         safe mode (default) */
    options_g[2] = 0;
    options_g[3] = 0;          /*                column compression (default) */

    jacval = nullptr;
    hessval = nullptr;
    sparse_jac(tag_g, m, n, 0, x0_.data(), &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

    nnz_jac_g = nnz_jac;

    options_L[0] = 0;
    options_L[1] = 1;

    sparse_hess(tag_L, n, 0, x0_.data(), &nnz_L, &rind_L, &cind_L, &hessval, options_L);
    nnz_h_lag = nnz_L;

    delete[] lam;
    delete[] g;
    delete[] xa;
    delete[] zu;
    delete[] zl;
    delete[] lamp;
  }

  std::vector<double> result_;

private:
  double w_inf_;
  int nfe_, disc_nvar_;
  int nvar_, nrows_, ncols_;
  const Constraints &profile_;
  const FullStates &guess_;
  const PlannerConfig &config_;

  std::vector<double> x0_;

  double *obj_lam;

  //** variables for sparsity exploitation
  unsigned int *rind_g;        /* row indices    */
  unsigned int *cind_g;        /* column indices */
  double *jacval;              /* values         */
  unsigned int *rind_L;        /* row indices    */
  unsigned int *cind_L;        /* column indices */
  double *hessval;             /* values */
  int nnz_jac;
  int nnz_L;
  int options_g[4];
  int options_L[4];
};

LightweightProblem::LightweightProblem(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env)
    : IOptimizer(std::move(config), std::move(env)) {
  app_.Options()->SetIntegerValue("print_level", 0);
  app_.Options()->SetIntegerValue("max_iter", config_->opti_inner_iter_max);
  app_.Options()->SetNumericValue("bound_relax_factor", 0.0);
  app_.Options()->SetStringValue("linear_solver", "mumps");

  auto status = app_.Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    ROS_ERROR("solver initialization failed");
    exit(-1);
  }
}

bool LightweightProblem::Solve(
    double w_inf, const Constraints &profile, const FullStates &guess, FullStates &result, double &infeasibility) {
  double solver_st = GetCurrentTimestamp();

  Ipopt::SmartPtr<LiomIPOPTInterface> interface = new LiomIPOPTInterface(w_inf, profile, guess, *config_);

  auto status = app_.OptimizeTNLP(interface);
  bool nlp_convergence = (status == Ipopt::Solve_Succeeded)
      || (status == Ipopt::Solved_To_Acceptable_Level)
      || (status == Ipopt::Feasible_Point_Found)
      || (status == Ipopt::Search_Direction_Becomes_Too_Small)
      || (status == Ipopt::Maximum_Iterations_Exceeded);
  if (!nlp_convergence) {
    return false;
  }

  result = ConvertVectorToStates(interface->result_.data(), guess.states.size(), profile.start, profile.goal);
  infeasibility = interface->eval_infeasibility(interface->result_.data());

  std::cout << "wall_t: " << GetCurrentTimestamp() - solver_st
    << ", status: " << status
    << ", infeasibility: " << infeasibility
    << ", tf: " << result.tf << std::endl;

  return true;
}


}