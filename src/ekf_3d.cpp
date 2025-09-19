#include "ekf_3d.h"

EKF_3D::EKF_3D(QObject *parent) : QObject{parent}
{
    x_hat.setZero();
    P_hat.setIdentity();
    pre_mo_tf.setIdentity();
    cur_tf.setIdentity();
}

EKF_3D::~EKF_3D()
{

}

void EKF_3D::init(const Eigen::Matrix4d& tf)
{
    // clear first
    reset();

    std::lock_guard<std::mutex> lock(mtx);

    // set state, covariance
    x_hat = TF_to_ZYX(tf);
    P_hat = Eigen::Matrix6d::Identity() * 1e-3;

    // set process noise, Q_k (odom)
    M_k = Eigen::Matrix6d::Zero();
    M_k(0,0) = 0.05*0.05;
    M_k(1,1) = 0.05*0.05;
    M_k(2,2) = 0.05*0.05;
    M_k(3,3) = (0.5*D2R) * (0.5*D2R);
    M_k(4,4) = (0.5*D2R) * (0.5*D2R);
    M_k(5,5) = (2.0*D2R) * (2.0*D2R);

    // set measurement noise, R_k (icp)
    R_k = Eigen::Matrix6d::Zero();
    R_k(0,0) = 0.01*0.01;
    R_k(1,1) = 0.01*0.01;
    R_k(2,2) = 0.01*0.01;
    R_k(3,3) = (0.05*D2R) * (0.05*D2R);
    R_k(4,4) = (0.05*D2R) * (0.05*D2R);
    R_k(5,5) = (1.0*D2R)  * (1.0*D2R);

    has_pre_mo_tf = false;

    initialized = true;

    printf("[EKF_3D] init (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f )\n",
           x_hat[0], x_hat[1], x_hat[2], x_hat[3]*R2D, x_hat[4]*R2D, x_hat[5]*R2D);
}

void EKF_3D::reset()
{
    x_hat.setZero();
    P_hat.setIdentity();
    pre_mo_tf.setIdentity();

    has_pre_mo_tf.store(false);
    initialized.store(false);
}

Eigen::Matrix4d EKF_3D::get_cur_tf()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Matrix4d res = ZYX_to_TF(x_hat);
    return res;
}

void EKF_3D::predict(const Eigen::Matrix4d& odom_tf)
{
    if(!initialized.load())
    {
        return;
    }

    if(!has_pre_mo_tf)
    {
        pre_mo_tf = odom_tf;
        has_pre_mo_tf = true;
        return;
    }

    Eigen::Matrix4d delta_tf = pre_mo_tf.inverse() * odom_tf;

    // based on body
    double delta_x = delta_tf(0,3);
    double delta_y = delta_tf(1,3);
    double delta_th = std::atan2(delta_tf(1,0), delta_tf(0,0));

    std::lock_guard<std::mutex> lock(mtx);

    double x  = x_hat(0);
    double y  = x_hat(1);
    double z  = x_hat(2);
    double phi = x_hat(3);
    double theta = x_hat(4);
    double psi0 = x_hat(5);
    double psi = psi0 + 0.5*delta_th;
    // double psi  = psi0;

    double cos_psi = std::cos(psi), sin_psi = std::sin(psi);
    double cos_theta = std::cos(theta), sin_theta = std::sin(theta);
    double cos_phi = std::cos(phi), sin_phi = std::sin(phi);

    // state prediction
    Eigen::Vector6d x_bar;
    x_bar(0) = x + (cos_psi*cos_theta)*delta_x + (cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*delta_y;
    x_bar(1) = y + (sin_psi*cos_theta)*delta_x + (sin_psi*sin_theta*sin_phi + cos_psi*cos_phi)*delta_y;
    x_bar(2) = z + (-sin_theta)*delta_x + (cos_theta*sin_phi)*delta_y;
    x_bar(3) = phi;
    x_bar(4) = theta;
    x_bar(5) = std::atan2(std::sin(psi0 + delta_th), std::cos(psi0 + delta_th));

    // jacobian F_k
    Eigen::Matrix6d F_k = Eigen::Matrix6d::Identity();
    F_k(0,3) = (cos_psi*sin_theta*cos_phi + sin_psi*sin_phi) * delta_y;
    F_k(0,4) = (-cos_psi*sin_theta) * delta_x + (cos_psi*cos_theta*sin_phi) * delta_y;
    F_k(0,5) = (-sin_psi*cos_theta) * delta_x + (-sin_psi*sin_theta*sin_phi - cos_psi*cos_phi) * delta_y;

    F_k(1,3) = (sin_psi*sin_theta*cos_phi - cos_psi*sin_phi) * delta_y;
    F_k(1,4) = (-sin_psi*sin_theta) * delta_x + (sin_psi*cos_theta*sin_phi) * delta_y;
    F_k(1,5) = ( cos_psi*cos_theta) * delta_x + ( cos_psi*sin_theta*sin_phi - sin_psi*cos_phi) * delta_y;

    F_k(2,3) = (cos_theta*cos_phi) * delta_y;
    F_k(2,4) = (-cos_theta) * delta_x + (-sin_theta*sin_phi) * delta_y;
    F_k(2,5) = 0.0;

    // jacobian L_k
    Eigen::Matrix6d L_k = Eigen::Matrix6d::Zero();
    L_k(0,0) = cos_psi*cos_theta;
    L_k(0,1) = cos_psi*sin_theta*sin_phi - sin_psi*cos_phi;
    L_k(0,2) = 0.0;

    L_k(1,0) = sin_psi*cos_theta;
    L_k(1,1) = sin_psi*sin_theta*sin_phi + cos_psi*cos_phi;
    L_k(1,2) = 0.0;

    L_k(2,0) = -sin_theta;
    L_k(2,1) = cos_theta*sin_phi;
    L_k(2,2) = 1.0;

    L_k(0,5) = 0.5 * ((-sin_psi*cos_theta) * delta_x + (-sin_psi*sin_theta*sin_phi - cos_psi*cos_phi) * delta_y);
    L_k(1,5) = 0.5 * (( cos_psi*cos_theta) * delta_x + ( cos_psi*sin_theta*sin_phi - sin_psi*cos_phi) * delta_y);
    L_k(2,5) = 0.0;

    L_k(3,3) = 1.0;
    L_k(4,4) = 1.0;
    L_k(5,5) = 1.0;

    // Q_k
    Eigen::Matrix6d Q_k = L_k * M_k * L_k.transpose();

    // covariance prediction
    Eigen::Matrix6d P_bar = F_k * P_hat * F_k.transpose() + Q_k;

    // commit
    x_hat = x_bar;
    P_hat = 0.5 * (P_bar + P_bar.transpose());
    // P_hat = P_bar;

    pre_mo_tf = odom_tf;

    // printf("[EKF_3D] prediction: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f )\n",
    //        x_hat[0], x_hat[1], x_hat[2], x_hat[3]*R2D, x_hat[4]*R2D, x_hat[5]*R2D);
}

/*
void EKF_3D::estimate(const Eigen::Matrix4d& icp_tf, const Eigen::Vector2d& ieir)
{
    if(!initialized.load())
    {
        return;
    }
    auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };


    // update measurement
    Eigen::Vector6d z_k = TF_to_ZYX(icp_tf);

    std::lock_guard<std::mutex> lock(mtx);

    // innovation y = z - h(x_bar)
    Eigen::Vector6d y_k = z_k - x_hat;
    y_k(3) = wrap(y_k(3));
    y_k(4) = wrap(y_k(4));
    y_k(5) = wrap(y_k(5));

    // innovation covariance
    Eigen::Matrix6d S_k = P_hat + R_k;

    // outlier detection - mahalanobis distance check (99% gate)
    // double d2 = y_k.transpose() * S_k.ldlt().solve(y_k);
    // if(d2 > 16.81)
    // {
    //     std::printf("[EKF_3D] ICP rejected, mahalanobis=%.2f\n", d2);
    //     return;
    // }

    // calc kalman gain
    Eigen::Matrix6d K_k = P_hat * S_k.ldlt().solve(Eigen::Matrix6d::Identity());

    // state estimation
    Eigen::Vector6d x_new = x_hat + K_k * y_k;
    x_new(3) = wrap(x_new(3));
    x_new(4) = wrap(x_new(4));
    x_new(5) = wrap(x_new(5));

    // error covariance update (joseph form)
    Eigen::Matrix6d I6 = Eigen::Matrix6d::Identity();
    Eigen::Matrix6d I_minus_K = I6 - K_k;
    Eigen::Matrix6d P_new = I_minus_K * P_hat * I_minus_K.transpose() + K_k * R_k * K_k.transpose();

    // commit
    x_hat = x_new;
    P_hat = 0.5 * (P_new + P_new.transpose());

    // debug
    printf("[EKF_3D] estimation:(%.3f, %.3f, %.3f, %.2f, %.2f, %.2f), innovation:(%.3f, %.3f, %.3f, %.2f, %.2f, %.2f), "
           "K_k:(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)\n",
           x_hat(0), x_hat(1), x_hat(2), x_hat(3)*R2D, x_hat(4)*R2D, x_hat(5)*R2D,
           y_k(0), y_k(1), y_k(2), y_k(3)*R2D, y_k(4)*R2D, y_k(5)*R2D,
           K_k(0,0), K_k(1,1), K_k(2,2), K_k(3,3), K_k(4,4), K_k(5,5));
}
*/

void EKF_3D::estimate(const Eigen::Matrix4d& icp_tf, const Eigen::Vector2d& ieir)
{
    if(!initialized.load())
    {
        return;
    }

    auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };

    auto RzRyRx = [](double ph,double th,double ps) -> Eigen::Matrix3d
    {
        double cph=std::cos(ph), sph=std::sin(ph);
        double cth=std::cos(th), sth=std::sin(th);
        double cps=std::cos(ps), sps=std::sin(ps);
        Eigen::Matrix3d Rx; Rx<<1,0,0, 0,cph,-sph, 0,sph,cph;
        Eigen::Matrix3d Ry; Ry<<cth,0,sth, 0,1,0, -sth,0,cth;
        Eigen::Matrix3d Rz; Rz<<cps,-sps,0, sps,cps,0, 0,0,1;
        return Rz*Ry*Rx;
    };

    auto rotToZYX = [](const Eigen::Matrix3d& R) -> Eigen::Vector3d
    {
        double th = std::asin(-R(2,0)); double ph, ps;
        if (std::fabs(std::cos(th))<1e-6){ ph=0.0; ps=std::atan2(-R(0,1), R(1,1)); }
        else { ph=std::atan2(R(2,1), R(2,2)); ps=std::atan2(R(1,0), R(0,0)); }
        return Eigen::Vector3d(ph, th, ps);
    };

    auto so3Log = [](const Eigen::Matrix3d& R) -> Eigen::Vector3d
    {
        double c = (R.trace()-1.0)*0.5; c = std::min(1.0,std::max(-1.0,c));
        double th = std::acos(c);
        if (th < 1e-8)
            return 0.5*Eigen::Vector3d(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
        if (M_PI - th < 1e-4){
            Eigen::Matrix3d RpI = R + Eigen::Matrix3d::Identity();
            Eigen::Vector3d v; int idx; RpI.diagonal().maxCoeff(&idx);
            v = RpI.col(idx); if (v.norm()<1e-8) v = RpI.col((idx+1)%3);
            v.normalize(); return th*v;
        }
        Eigen::Matrix3d K = (R - R.transpose())*(0.5/std::sin(th));
        return th*Eigen::Vector3d(K(2,1), K(0,2), K(1,0));
    };

    auto so3Exp = [](const Eigen::Vector3d& w) -> Eigen::Matrix3d
    {
        const double th = w.norm();
        Eigen::Matrix3d W;
        W << 0, -w(2), w(1),
             w(2), 0, -w(0),
            -w(1), w(0), 0;
        double A, B; // R = I + A*W + B*W^2
        if (th < 1e-8) { const double th2=th*th; A=1.0 - th2/6.0; B=0.5 - th2/24.0; }
        else { A = std::sin(th)/th; B = (1.0-std::cos(th))/(th*th); }
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R.noalias() += A*W + B*(W*W);
        return R;
    };

    // update measurement
    Eigen::Matrix3d R_meas = icp_tf.block(0,0,3,3);
    Eigen::Vector3d t_meas(icp_tf(0,3), icp_tf(1,3), icp_tf(2,3));
    if(R_meas.determinant() < 0.0)
    {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_meas, Eigen::ComputeFullU|Eigen::ComputeFullV);
        Eigen::Matrix3d U=svd.matrixU(), V=svd.matrixV();
        Eigen::Matrix3d S=Eigen::Matrix3d::Identity(); S(2,2)=(U*V.transpose()).determinant();
        R_meas = U*S*V.transpose();
    }

    std::lock_guard<std::mutex> lock(mtx);

    // prediction rotation matrix
    double ph = x_hat(3), th = x_hat(4), ps = x_hat(5);
    Eigen::Matrix3d R_prediction = RzRyRx(ph, th, ps);

    // innovation
    Eigen::Vector6d y_k;
    y_k.block(0,0,3,1) = t_meas - x_hat.block(0,0,3,1);
    Eigen::Matrix3d dR = R_meas * R_prediction.transpose();
    y_k.block(3,0,3,1) = so3Log(dR);

    // covariance / kalman gain ---
    Eigen::Matrix6d S_k = P_hat + R_k;
    Eigen::Matrix6d K_k = P_hat * S_k.ldlt().solve(Eigen::Matrix6d::Identity());

    // state estimation [dx, dy, dz, dωx, dωy, dωz]^T
    Eigen::Vector6d delta = K_k * y_k;
    Eigen::Vector6d x_new = x_hat;
    x_new.block(0,0,3,1) += delta.block(0,0,3,1);
    Eigen::Matrix3d R_new = so3Exp(delta.block(3,0,3,1)) * R_prediction;

    // matrix -> euler
    Eigen::Vector3d eul = rotToZYX(R_new);
    Eigen::Vector3d eul_flip(eul[0]+M_PI, -eul[1], eul[2]+M_PI);
    auto score = [&](const Eigen::Vector3d& a)
    {
        Eigen::Vector3d d = a - x_hat.block(3,0,3,1);
        return std::pow(wrap(d[0]),2)+std::pow(wrap(d[1]),2)+std::pow(wrap(d[2]),2);
    };
    Eigen::Vector3d eul_cont = (score(eul_flip) < score(eul)) ? eul_flip : eul;
    for(int i = 0; i < 3; i++)
    {
        eul_cont[i] = x_hat(3+i) + wrap(eul_cont[i]-x_hat(3+i));
    }

    x_new(3) = eul_cont[0];
    x_new(4) = eul_cont[1];
    x_new(5) = eul_cont[2];

    // error covariance update (joseph form)
    Eigen::Matrix6d I6 = Eigen::Matrix6d::Identity();
    Eigen::Matrix6d IminusK = I6 - K_k;
    Eigen::Matrix6d P_new = IminusK * P_hat * IminusK.transpose() + K_k * R_k * K_k.transpose();

    // commit
    x_hat = x_new;
    P_hat = 0.5 * (P_new + P_new.transpose());

    // debug
    Eigen::Matrix3d dR_after = R_meas * RzRyRx(x_hat(3),x_hat(4),x_hat(5)).transpose();
    Eigen::Vector3d w_after  = so3Log(dR_after);

    printf("[EKF_3D] estimation:(%.3f, %.3f, %.3f, %.2f, %.2f, %.2f), "
           "innovation:(%.3f, %.3f, %.3f, %.2f, %.2f, %.2f), "
           "rot_err_after:(%.2f, %.2f, %.2f deg), "
           "K_k:(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)\n",
           x_hat(0), x_hat(1), x_hat(2),
           x_hat(3)*R2D, x_hat(4)*R2D, x_hat(5)*R2D,
           y_k(0), y_k(1), y_k(2),
           y_k(3)*R2D, y_k(4)*R2D, y_k(5)*R2D,
           w_after(0)*R2D, w_after(1)*R2D, w_after(2)*R2D,
           K_k(0,0), K_k(1,1), K_k(2,2), K_k(3,3), K_k(4,4), K_k(5,5));
}
