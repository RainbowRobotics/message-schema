#ifndef UTILS_CPP
#define UTILS_CPP

#include "utils.h"

double st_time_for_get_time = get_time();
double get_time()
{
    std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    return (timestamp*1.0e-9) - st_time_for_get_time;
}

QString get_time_str()
{
    QString str = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh:mm:ss.zzz");
    str.replace("-", "_");
    str.replace(":", "_");
    str.replace(".", "_");
    return str;
}

Eigen::Matrix4d ZYX_to_TF(double tx, double ty, double tz, double rx, double ry, double rz)
{
    Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    Eigen::Matrix4d res;
    res.setIdentity();

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = tx;
    res(1,3) = ty;
    res(2,3) = tz;
    res(3,3) = 1;
    return res;
}

Eigen::Matrix4d ZYX_to_TF(QString str)
{
    // parsing z-y-x euler "tx,ty,tz,rx,ry,rz" (meter, deg)
    QStringList list = str.split(",");
    if(list.size() != 6)
    {
        printf("string invalid\n");
        return Eigen::Matrix4d::Identity();
    }

    double tx = list[0].toDouble();
    double ty = list[1].toDouble();
    double tz = list[2].toDouble();
    double rx = list[3].toDouble()*D2R;
    double ry = list[4].toDouble()*D2R;
    double rz = list[5].toDouble()*D2R;

    Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    Eigen::Matrix4d res;
    res.setIdentity();

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = tx;
    res(1,3) = ty;
    res(2,3) = tz;
    return res;
}

Eigen::Matrix4d ZYX_to_TF(Sophus::Vector6d zyx)
{
    double tx = zyx[0];
    double ty = zyx[1];
    double tz = zyx[2];
    double rx = zyx[3];
    double ry = zyx[4];
    double rz = zyx[5];

    Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    Eigen::Matrix4d res;
    res.setIdentity();

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = tx;
    res(1,3) = ty;
    res(2,3) = tz;
    return res;
}

Sophus::Vector6d TF_to_ZYX(Eigen::Matrix4d tf)
{
    Eigen::Matrix3d R = tf.block(0,0,3,3);
    Eigen::Vector3d t = tf.block(0,3,3,1);
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);

    Sophus::Vector6d res;    
    res[0] = t[0];
    res[1] = t[1];
    res[2] = t[2];
    res[3] = euler[2];
    res[4] = euler[1];
    res[5] = euler[0];
    return res;
}

Eigen::Vector3d TF_to_se2(Eigen::Matrix4d tf)
{
    Eigen::Vector3d t = tf.block<3,1>(0,3);
    double rz = std::atan2(tf(1, 0), tf(0, 0));

    Eigen::Vector3d res;
    res[0] = t[0];
    res[1] = t[1];
    res[2] = rz;
    return res;
}

Eigen::Matrix4d se2_to_TF(Eigen::Vector3d xi)
{
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

    tf(0, 3) = xi[0];
    tf(1, 3) = xi[1];

    double rz = xi[2];
    tf(0, 0) = cos(rz);
    tf(0, 1) = -sin(rz);
    tf(1, 0) = sin(rz);
    tf(1, 1) = cos(rz);

    return tf;
}

Eigen::Matrix4d se3_to_TF(Sophus::Vector6d xi)
{
    return Sophus::SE3d::exp(xi).matrix();
}

Sophus::Vector6d TF_to_se3(Eigen::Matrix4d tf)
{
    return Sophus::SE3d(tf).log();
}

Eigen::Matrix4d string_to_TF(QString str)
{
    // string is ZYX euler "tx,ty,tz,rx,ry,rz" (meter, deg)
    QStringList list = str.split(",");
    if(list.size() != 6)
    {
        printf("string invalid\n");
        return Eigen::Matrix4d::Identity();
    }

    double tx = list[0].toDouble();
    double ty = list[1].toDouble();
    double tz = list[2].toDouble();
    double rx = list[3].toDouble()*D2R;
    double ry = list[4].toDouble()*D2R;
    double rz = list[5].toDouble()*D2R;

    Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    Eigen::Matrix4d res;
    res.setIdentity();

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = tx;
    res(1,3) = ty;
    res(2,3) = tz;
    return res;
}

QString TF_to_string(Eigen::Matrix4d TF)
{
    // string is ZYX euler "tx,ty,tz,rx,ry,rz" (meter, deg)
    double tx = TF(0,3);
    double ty = TF(1,3);
    double tz = TF(2,3);

    Eigen::Matrix3d R = TF.block(0,0,3,3);
    Eigen::Vector3d euler = R.eulerAngles(2,1,0);

    double rx = euler[2]*R2D;
    double ry = euler[1]*R2D;
    double rz = euler[0]*R2D;

    QString res;
    res.sprintf("%f,%f,%f,%f,%f,%f", tx, ty, tz, rx, ry, rz);
    return res;
}

double calc_seg_dist(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P)
{
    Eigen::Vector3d ab = P1-P0;
    Eigen::Vector3d av = P-P0;
    if(av.dot(ab) <= 0)
    {
        return av.norm();
    }

    Eigen::Vector3d bv = P-P1;
    if(bv.dot(ab) >= 0)
    {
        return bv.norm();
    }

    return (ab.cross(av)).norm() / ab.norm();
}

Eigen::Matrix4d intp_tf(double alpha, Eigen::Matrix4d tf0, Eigen::Matrix4d tf1)
{
    Eigen::Matrix4d dG = tf0.inverse()*tf1;

    Sophus::Vector6d dxi = Sophus::SE3d(dG).log();
    dxi[0] *= alpha;
    dxi[1] *= alpha;
    dxi[2] *= alpha;
    dxi[3] *= alpha;
    dxi[4] *= alpha;
    dxi[5] *= alpha;

    Eigen::Matrix4d res = tf0*Sophus::SE3d::exp(dxi).matrix();
    return res;
}

void refine_pose(Eigen::Matrix4d& G)
{
    Eigen::Matrix3d R = G.block(0,0,3,3);
    G.block(0,0,3,3) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
}

bool compare_view_vector(Eigen::Vector3d V0, const Eigen::Vector3d V1, double threshold)
{
    double angle = std::acos(V0.dot(V1));
    if (angle > threshold)
    {
        return true;
    }
    return false;
}

std::vector<Eigen::Vector3d> transform_pts(std::vector<Eigen::Vector3d> &pts, Eigen::Matrix4d tf)
{
    std::vector<Eigen::Vector3d> res;
    for(size_t p = 0; p < pts.size(); p++)
    {
        Eigen::Vector3d P = tf.block(0,0,3,3)*pts[p] + tf.block(0,3,3,1);
        res.push_back(P);
    }
    return res;
}

double saturation(double val, double min, double max)
{
    if(val < min)
    {
        val = min;
    }
    else if(val > max)
    {
        val = max;
    }

    return val;
}

#endif // UTILS_CPP
