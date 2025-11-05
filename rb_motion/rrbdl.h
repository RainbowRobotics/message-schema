#ifndef RRBDL_H
#define RRBDL_H

#include <rbdl/rbdl.h>

#include "rmath.h"
#include "rquadprog.h"
#include "system.h"


enum class IkMode   {IK_GENERAL, IK_STRICT};
enum class IkRet    {IK_OK, IK_FAIL_NAN, IK_FAIL_TOSMALL, IK_FAIL_ZERO, IK_FAIL_DQ_NAN, IK_FAIL_POS_ERR, IK_FAIL_ROT_ERR, IK_FAIL_ELBOW_ST, IK_UNSOLV};

struct IkResult{
    IkRet       result;
    double      time_scaler;
    double      error_size[2];

    VectorJd    q_out_deg;
};

class RBDLrobot {
public:
    RBDLrobot();
    ~RBDLrobot();

    int                 Init_Robot(ROBOT_CONFIG rConfig, TCP_CONFIG tConfig, Vector3d tGravity);
    int                 Update_Robot(ROBOT_CONFIG rConfig, TCP_CONFIG tConfig, Vector3d tGravity);

    VectorCd            Calc_ForwardKinematics(const Eigen::VectorXd& q);
    VectorJd            Calc_InverseDynamics(const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& dq,
                                     const Eigen::VectorXd& ddq);

    Eigen::MatrixXd     Calc_Jacobian(const Eigen::VectorXd& q);
    double              Calc_Jacobian_Det(const Eigen::VectorXd& q);
    double              Calc_Redundancy(const Eigen::VectorXd& q_ang);
    Eigen::MatrixXd     Calc_MassMatrix(const Eigen::VectorXd& q);

    double              Calc_MaxVel(double t_jacob_det, double original_MaxVel_deg);
    IkResult            Calc_InverseKinematics_Loop(IkMode mode, double dt_sec, VectorCd target, VectorCd current
                                    , VectorJd q_begin, VectorJd q_min, VectorJd q_max, VectorJd dq_max, VectorJd ddq_max, VectorJd prev_dq_deg
                                    , double err_bound_pos, double err_bound_ori, int is_firstLoop);
    IkResult            Calc_InverseKinematics_Static(VectorCd target, int iter_traj, int iter_loop, VectorCd start_X, VectorJd start_J, VectorJd q_min, VectorJd q_max);
    IkResult            Calc_InverseKinematics_Static(VectorCd target, int iter_traj, int iter_loop, VectorCd start_X, VectorJd start_J);

private:
    RigidBodyDynamics::Model    *model_;

    Eigen::MatrixXd             jacob_matrix = Eigen::MatrixXd::Zero(6, NO_OF_JOINT);
    double                      jacob_determinant;
    double                      jacob_determinant_old;
    
    double                      ik_speed_scaler_local;
    double                      ik_speed_scaler_jacob;
    double                      ik_speed_scaler_jacob_input;

    RBQuadProg                  ik_quad_prog;

    MatrixNd                    ik_Aineq_ub = MatrixNd::Identity(NO_OF_JOINT, NO_OF_JOINT);
    MatrixNd                    ik_Aineq_lb = -1.0 * MatrixNd::Identity(NO_OF_JOINT, NO_OF_JOINT);
    VectorNd                    ik_Bineq_ub = VectorNd::Zero(NO_OF_JOINT, 1);
    VectorNd                    ik_Bineq_lb = VectorNd::Zero(NO_OF_JOINT, 1);

    MatrixNd                    ik_Aineq = MatrixNd::Zero(NO_OF_JOINT * 2, NO_OF_JOINT);
    VectorNd                    ik_Bineq = VectorNd::Zero(NO_OF_JOINT * 2, 1);

    MatrixNd                    ik_Aeq = MatrixNd::Zero(1, NO_OF_JOINT);
    MatrixNd                    ik_Beq = MatrixNd::Zero(1, 1);

    MatrixNd                    ik_Abehav = MatrixNd::Zero(6, NO_OF_JOINT);
    VectorNd                    ik_Bbehav = VectorNd::Zero(6, 1);//modong




    Body                        r_bodys[NO_OF_JOINT];
    RigidBodyDynamics::Joint    r_joints[NO_OF_JOINT];
    int                         r_nums[NO_OF_JOINT];
    double                      r_masss[NO_OF_JOINT];
    Vector3d                    r_coms[NO_OF_JOINT];
    Matrix3d                    r_inertias[NO_OF_JOINT];

    Body                        b_ee;
    RigidBodyDynamics::Joint    j_ee;
    int                         n_ee;
    unsigned int                ee_body;  // End-effector
    double                      m_ee;
    Vector3d                    c_ee;
    Matrix3d                    I_ee;


    VectorJd                    default_joint_lim_low;
    VectorJd                    default_joint_lim_up;

    int                         redundancy_mode;
};

#endif // RRBDL_H
