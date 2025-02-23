#ifndef UTILS_CPP
#define UTILS_CPP

#include "my_utils.h"

cv::Vec3b colors[10] =
{
    cv::Vec3b(0, 0, 255),     // 밝은 빨간색 (BGR: Blue, Green, Red)
    cv::Vec3b(0, 128, 0),     // 진한 녹색
    cv::Vec3b(255, 0, 0),     // 밝은 파란색
    cv::Vec3b(0, 165, 255),   // 주황색
    cv::Vec3b(255, 0, 255),   // 마젠타
    cv::Vec3b(128, 128, 128), // 회색
    cv::Vec3b(0, 255, 255),   // 주황색
    cv::Vec3b(147, 20, 255),  // 핑크색
    cv::Vec3b(255, 255, 0),   // 밝은 하늘색
    cv::Vec3b(0, 255, 0)      // 밝은 녹색
};

std::vector<VoltageCapacity> volt_lookup_data =
{
    {42.84, 1}, {43.54, 2}, {44.52, 3}, {44.8, 4}, {45.08, 5}, {45.36, 6},
    {46.06, 7}, {46.48, 8}, {46.76, 9}, {47.04, 10}, {47.32, 11}, {47.46, 12},
    {47.74, 13}, {48.02, 14}, {48.3, 15}, {48.44, 16}, {48.58, 17}, {48.72, 18},
    {48.79, 19}, {48.86, 20}, {49, 21}, {49.07, 22}, {49.14, 23}, {49.28, 24},
    {49.42, 25}, {49.56, 26}, {49.7, 27}, {49.84, 28}, {49.91, 29}, {49.98, 30},
    {50.12, 31}, {50.26, 32}, {50.4, 33}, {50.54, 34}, {50.61, 35}, {50.68, 36},
    {50.75, 37}, {50.82, 38}, {50.89, 39}, {50.96, 40}, {51.03, 41}, {51.1, 42},
    {51.142, 43}, {51.198, 44}, {51.24, 45}, {51.31, 46}, {51.38, 47}, {51.45, 48},
    {51.52, 49}, {51.66, 50}, {51.73, 51}, {51.8, 52}, {51.94, 53}, {52.08, 54},
    {52.22, 55}, {52.36, 56}, {52.5, 57}, {52.78, 58}, {52.92, 59}, {53.06, 60},
    {53.2, 61}, {53.34, 62}, {53.48, 63}, {53.62, 64}, {53.76, 65}, {53.9, 66},
    {54.04, 67}, {54.18, 68}, {54.32, 69}, {54.46, 70}, {54.53, 71}, {54.6, 72},
    {54.74, 73}, {55.02, 74}, {55.3, 75}, {55.37, 76}, {55.44, 77}, {55.51, 78},
    {55.58, 79}, {55.72, 80}, {55.86, 81}, {56, 82}, {56.14, 83}, {56.28, 84},
    {56.42, 85}, {56.56, 86}, {56.7, 87}, {56.98, 88}, {57.12, 89}, {57.26, 90},
    {57.33, 91}, {57.4, 92}, {57.47, 93}, {57.54, 94}, {57.68, 95}, {57.82, 96},
    {58.1, 97}, {58.24, 98}, {58.52, 99}, {58.8, 100}
};

QString AUTO_FSM_STATE_STR[7] =
{
    "AUTO_FSM_FIRST_ALIGN",
    "AUTO_FSM_DRIVING",
    "AUTO_FSM_FINAL_ALIGN",
    "AUTO_FSM_OBS",
    "AUTO_FSM_COMPLETE",
    "AUTO_FSM_DOCKING",
    "AUTO_FSM_PAUSE"
};

QString DOCK_FSM_STATE_STR[5] =
{
    "DOCK_FSM_DRIVING",
    "DOCK_FSM_DRIVING_FOR_CHRGE",
    "DOCK_FSM_WAIT_FOR_CHRGE",
    "DOCK_FSM_COMPLETE",
    "DOCK_FSM_FAILED"
};

bool ping(std::string ip_address)
{
    std::string command = "ping -c 1 -W 1 " + ip_address + " > /dev/null 2>&1";
    int result = system(command.c_str());
    return (result == 0);
}

double get_time()
{
    std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    return (timestamp*1.0e-9);
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
    res[3] = euler[2]; // rx
    res[4] = euler[1]; // ry
    res[5] = euler[0]; // rz
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
    tf(0, 0) = std::cos(rz);
    tf(0, 1) = -std::sin(rz);
    tf(1, 0) = std::sin(rz);
    tf(1, 1) = std::cos(rz);

    return tf;
}

Eigen::Matrix4d se3_to_TF(Sophus::Vector6d xi)
{
    return Sophus::SE3d::exp(xi).matrix();
}

Sophus::Vector6d TF_to_se3(Eigen::Matrix4d tf)
{
    return Sophus::SE3d::fitToSE3(tf).log();
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

Eigen::Matrix4d intp_tf(double alpha, Eigen::Matrix4d tf0, Eigen::Matrix4d tf1)
{
    if(alpha == 0.0)
    {
        return tf0;
    }
    else if(alpha == 1.0)
    {
        return tf1;
    }

    alpha = saturation(alpha, 0.0, 1.0);
    return Sophus::interpolate<Sophus::SE3d>(Sophus::SE3d::fitToSE3(tf0), Sophus::SE3d::fitToSE3(tf1), alpha).matrix();
}

std::vector<Eigen::Matrix4d> intp_tf(Eigen::Matrix4d tf0, Eigen::Matrix4d tf1, double dist_step, double th_step)
{
    Eigen::Quaterniond q0(tf0.block<3,3>(0,0));
    Eigen::Quaterniond q1(tf1.block<3,3>(0,0));

    Eigen::Vector3d t0 = tf0.block<3,1>(0,3);
    Eigen::Vector3d t1 = tf1.block<3,1>(0,3);

    double distance = (t1 - t0).norm();
    double theta = q0.angularDistance(q1);

    int dist_steps = std::max(1, static_cast<int>(distance / dist_step));
    int rot_steps = std::max(1, static_cast<int>(theta / th_step));
    int steps = std::max(dist_steps, rot_steps);

    std::vector<Eigen::Matrix4d> res;
    res.push_back(tf0);
    for(int i = 0; i < steps; ++i)
    {
        double alpha = static_cast<double>(i) / steps;

        Eigen::Vector3d t_interpolated = (1.0 - alpha) * t0 + alpha * t1;
        Eigen::Quaterniond q_interpolated = q0.slerp(alpha, q1);

        Eigen::Matrix4d tf_interpolated = Eigen::Matrix4d::Identity();
        tf_interpolated.block<3,3>(0,0) = q_interpolated.toRotationMatrix();
        tf_interpolated.block<3,1>(0,3) = t_interpolated;

        res.push_back(tf_interpolated);
    }
    res.push_back(tf1);

    return res;
}

std::vector<Eigen::Vector3d> intp_pts(Eigen::Vector3d P0, Eigen::Vector3d P1, double step)
{
    double distance = (P1 - P0).norm();
    Eigen::Vector3d normal = (P1 - P0).normalized();

    std::vector<Eigen::Vector3d> res;
    res.push_back(P0);

    for(double d = step; d <= distance - step; d += step)
    {
        Eigen::Vector3d P = P0 + d*normal;
        res.push_back(P);
    }

    res.push_back(P1);
    return res;
}

void refine_pose(Eigen::Matrix4d& G)
{
    G = Sophus::SE3d::fitToSE3(G).matrix();
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

double sgn(double val)
{
    if(val >= 0.0)
    {
        return 1.0;
    }
    else
    {
        return -1.0;
    }
}

int saturation(int val, int min, int max)
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

double calc_seg_dist(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P)
{
    Eigen::Vector3d ab = P1-P0;
    Eigen::Vector3d av = P-P0;
    if(av.dot(ab) <= 0.0)
    {
        return av.norm();
    }

    Eigen::Vector3d bv = P-P1;
    if(bv.dot(ab) >= 0.0)
    {
        return bv.norm();
    }

    return (ab.cross(av)).norm() / ab.norm();
}

bool check_point_on_segment(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P)
{
    Eigen::Vector3d P0P1 = P1 - P0;
    Eigen::Vector3d P0P = P - P0;

    double lengthSquared = P0P1.squaredNorm();
    if(lengthSquared == 0.0)
    {
        return false;
    }

    double t = P0P.dot(P0P1) / lengthSquared;
    if(t >= 0.0 && t <= 1.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double calc_motion_time(double _s, double _v0, double _v1, double _acc)
{
    if(std::abs(_s) == 0)
    {
        return 0;
    }

    double sign = _v0*_v1;
    if(sign >= 0)
    {
        // v0, v1 same sign
        double v0 = std::abs(_v0);
        double v1 = std::abs(_v1);
        double s = std::abs(_s);
        double acc;
        if(v1 - v0 > 0)
        {
            // accel
            acc = _acc;
        }
        else if(v1 - v0 < 0)
        {
            // decel
            acc = -_acc;
        }
        else
        {
            // constant case
            double t = s/v0;
            return t;
        }

        double t0 = (v1-v0)/acc;
        double s0 = v0*t0+0.5*acc*t0*t0;
        if(s0 >= s)
        {
            double a = 0.5*acc;
            double b = v0;
            double c = -s;

            // always b^2 - 4ac > 0, because c is minus
            double tmp0 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
            double tmp1 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
            double t = std::max<double>(tmp0, tmp1);
            return t;
        }

        double t1 = (s - v0*t0 - 0.5*(v1-v0)*t0 + v1*t0)/v1;
        return t1;
    }
    else
    {
        // v0, v1 different sign
        double v0 = std::abs(_v0);
        double v1 = std::abs(_v1);
        double s = std::abs(_s);
        double acc = std::abs(_acc);

        double t0 = v0/acc;
        double t1 = t0 + v1/acc;

        double s1 = -0.5*v0*t0 + 0.5*v1*(t1-t0);
        if(s1 >= s)
        {
            v0 *= -1;
            double a = 0.5*acc;
            double b = v0;
            double c = -s;

            // always b^2 - 4ac > 0, because c is minus
            double tmp0 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
            double tmp1 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
            double t = std::max<double>(tmp0, tmp1);
            return t;
        }

        double t2 = (s + 0.5*v0*t0 - 0.5*v1*(t1-t0) + v1*t1)/v1;
        return t2;
    }
}

double calc_dist_2d(Eigen::Vector3d P)
{
    return std::sqrt(P[0]*P[0] + P[1]*P[1]);
}

double calc_cte(std::vector<Eigen::Matrix4d>& src, Eigen::Vector3d pos)
{
    int min_idx = 0;
    double min_d = 99999999;
    for(size_t p = 0; p < src.size(); p++)
    {
        double d = calc_dist_2d(src[p].block(0,3,3,1) - pos);
        if(d < min_d)
        {
            min_d = d;
            min_idx = p;
        }
    }

    Eigen::Matrix4d G = src[min_idx].inverse();
    Eigen::Vector3d P = G.block(0,0,3,3)*pos + G.block(0,3,3,1);
    return -P[1];
}

double calc_dth(const Eigen::Matrix4d& G0, const Eigen::Matrix4d& G1)
{
    return Eigen::Quaterniond(Eigen::Matrix3d(G1.block(0,0,3,3))).angularDistance(Eigen::Quaterniond(Eigen::Matrix3d(G0.block(0,0,3,3))));
}

std::vector<cv::Vec2i> line_iterator(cv::Vec2i pt0, cv::Vec2i pt1)
{
    std::vector<cv::Vec2i> res;

    int x1 = pt0[0];
    int y1 = pt0[1];
    int x2 = pt1[0];
    int y2 = pt1[1];

    int add_x = 0;
    int add_y = 0;
    int count = 0;

    int dx = x2-x1;
    int dy = y2-y1;

    if(dx < 0)
    {
        add_x = -1;
        dx = -dx;
    }
    else
    {
        add_x = 1;
    }

    if(dy < 0)
    {
        add_y = -1;
        dy = -dy;
    }
    else
    {
        add_y = 1;
    }

    int x = x1;
    int y = y1;

    if(dx >= dy)
    {
        for(int i = 0; i < dx; i++)
        {
            x += add_x;
            count += dy;

            if(count >= dx)
            {
                y += add_y;
                count -= dx;
            }

            res.push_back(cv::Vec2i(x,y));
        }
    }
    else
    {
        for(int i = 0; i < dy; i++)
        {
            y += add_y;
            count += dx;

            if(count >= dy)
            {
                x += add_x;
                count -= dy;
            }

            res.push_back(cv::Vec2i(x,y));
        }
    }

    return res;
}

std::vector<cv::Vec2i> circle_iterator(cv::Vec2i pt, int r)
{
    int x = r;
    int y = 0;
    int error = 3 - 2*r;

    std::vector<cv::Vec2i> res;
    while (x >= y)
    {
        res.push_back(cv::Vec2i(x, y) + pt);
        res.push_back(cv::Vec2i(x, -y) + pt);
        res.push_back(cv::Vec2i(-x, y) + pt);
        res.push_back(cv::Vec2i(-x, -y) + pt);

        res.push_back(cv::Vec2i(y, x) + pt);
        res.push_back(cv::Vec2i(y, -x) + pt);
        res.push_back(cv::Vec2i(-y, x) + pt);
        res.push_back(cv::Vec2i(-y, -x) + pt);

        if (error > 0)
        {
            error -= 4 * (--x);
        }
        error += 4 * (++y) + 2;
    }

    return res;
}

std::vector<cv::Vec2i> filled_circle_iterator(cv::Vec2i pt, int r)
{
    std::vector<cv::Vec2i> res;
    for(int i = -r; i <= r; i++)
    {
        for(int j = -r; j <= r; j++)
        {
            double d = i*i + j*j;
            if(d <= r*r)
            {
                res.push_back(cv::Vec2i(i,j) + pt);
            }
        }
    }
    return res;
}

std::vector<Eigen::Vector3d> circle_iterator_3d(Eigen::Vector3d center, double radius)
{
    std::vector<Eigen::Vector3d> res;
    for (int angle = 0; angle < 360; angle += 10)
    {
        double radians = angle * D2R;
        double x = center.x() + radius * std::cos(radians);
        double y = center.y() + radius * std::sin(radians);
        double z = center.z();
        res.push_back(Eigen::Vector3d(x, y, z));
    }
    return res;
}

pcl::PolygonMesh make_donut(double donut_radius, double tube_radius, Eigen::Matrix4d tf, double r, double g, double b, double a, int num_segments)
{
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.width = num_segments * num_segments;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int i = 0; i < num_segments; ++i)
    {
        double theta = 2.0 * M_PI * i / num_segments;
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);

        for (int j = 0; j < num_segments; ++j)
        {
            double phi = 2.0 * M_PI * j / num_segments;
            double cos_phi = std::cos(phi);
            double sin_phi = std::sin(phi);

            pcl::PointXYZRGBA point;
            point.x = (donut_radius + tube_radius * cos_phi) * cos_theta;
            point.y = (donut_radius + tube_radius * cos_phi) * sin_theta;
            point.z = tube_radius * sin_phi;
            point.r = static_cast<uint8_t>(r * 255);
            point.g = static_cast<uint8_t>(g * 255);
            point.b = static_cast<uint8_t>(b * 255);
            point.a = static_cast<uint8_t>(a * 255);

            cloud.points[i * num_segments + j] = point;
        }
    }

    pcl::transformPointCloud(cloud, cloud, tf);
    pcl::toPCLPointCloud2(cloud, mesh.cloud);

    mesh.polygons.resize(num_segments * num_segments);
    for (int i = 0; i < num_segments; ++i)
    {
        for (int j = 0; j < num_segments; ++j)
        {
            pcl::Vertices vertices;
            vertices.vertices.push_back((i * num_segments + j) % (num_segments * num_segments));
            vertices.vertices.push_back(((i + 1) * num_segments + j) % (num_segments * num_segments));
            vertices.vertices.push_back(((i + 1) * num_segments + (j + 1)) % (num_segments * num_segments));
            vertices.vertices.push_back((i * num_segments + (j + 1)) % (num_segments * num_segments));
            mesh.polygons[i * num_segments + j] = vertices;
        }
    }

    return mesh;
}

Eigen::Matrix4d calc_tf(Eigen::Vector3d P0, Eigen::Vector3d P1)
{
    if(P0.isApprox(P1))
    {
        printf("check calc_tf same points\n");
        Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
        res.block(0,3,3,1) = P1;
        return res;
    }
    else
    {
        Eigen::Vector3d x_axis = (P1 - P0).normalized();
        Eigen::Vector3d temp_z_axis(0, 0, 1);

        Eigen::Vector3d y_axis = temp_z_axis.cross(x_axis).normalized();
        Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix.col(0) = x_axis;
        rotation_matrix.col(1) = y_axis;
        rotation_matrix.col(2) = z_axis;

        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
        transformation_matrix.block<3, 1>(0, 3) = P0;
        return transformation_matrix;
    }
}

std::vector<Eigen::Matrix4d> calc_path_tf(std::vector<Eigen::Vector3d>& pos)
{
    std::vector<Eigen::Matrix4d> res;
    for(size_t p = 0; p < pos.size()-1; p++)
    {
        Eigen::Matrix4d tf = calc_tf(pos[p], pos[p+1]);
        res.push_back(tf);
    }

    Eigen::Matrix4d last_tf = res.back();
    last_tf.block(0,3,3,1) = pos.back();

    res.push_back(last_tf);
    return res;
}

std::vector<Eigen::Matrix4d> reorientation_path(std::vector<Eigen::Matrix4d>& path)
{
    if(path.size() < 2)
    {
        return path;
    }

    std::vector<Eigen::Matrix4d> res;
    for(size_t p = 0; p < path.size()-1; p++)
    {
        Eigen::Matrix4d tf0 = path[p];
        Eigen::Matrix4d tf1 = path[p+1];

        Eigen::Vector3d pos0 = tf0.block(0,3,3,1);
        Eigen::Vector3d pos1 = tf1.block(0,3,3,1);
        if(calc_dist_2d(pos1-pos0) < 0.05)
        {
            if(res.size() == 0)
            {
                res.push_back(tf0);
            }
            else
            {
                Eigen::Matrix4d tf = res.back();
                tf.block(0,3,3,1) = pos0;
                res.push_back(tf);
            }
        }
        else
        {
            Eigen::Matrix4d tf = calc_tf(pos0, pos1);
            res.push_back(tf);
        }
    }

    Eigen::Matrix4d tf = res.back();
    tf.block(0,3,3,1) = path.back().block(0,3,3,1);
    res.push_back(tf);
    return res;
}

Eigen::Matrix4d flip_lidar_tf(Eigen::Matrix4d tf)
{
    Eigen::Matrix4d reverse;
    reverse.setIdentity();

    reverse(1,1) = -1;
    reverse(2,2) = -1;

    Eigen::Matrix4d ref = tf*reverse;
    return ref;
}

Eigen::Matrix3d remove_rz(const Eigen::Matrix3d& rotation_matrix)
{
    Eigen::Matrix3d result = rotation_matrix;

    // Yaw 요소 제거 (z축 회전 요소를 제거)
    result(0, 0) = std::sqrt(rotation_matrix(0, 0) * rotation_matrix(0, 0) + rotation_matrix(1, 0) * rotation_matrix(1, 0));
    result(0, 1) = 0;
    result(0, 2) = rotation_matrix(0, 2);
    result(1, 0) = 0;
    result(1, 1) = std::sqrt(rotation_matrix(0, 0) * rotation_matrix(0, 0) + rotation_matrix(1, 0) * rotation_matrix(1, 0));
    result(1, 2) = rotation_matrix(1, 2);
    result(2, 0) = rotation_matrix(2, 0);
    result(2, 1) = rotation_matrix(2, 1);
    result(2, 2) = rotation_matrix(2, 2);

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(result, Eigen::ComputeFullU | Eigen::ComputeFullV);
    result = svd.matrixU() * svd.matrixV().transpose();
    return result;
}

Eigen::Vector2d dTdR(Eigen::Matrix4d G0, Eigen::Matrix4d G1)
{
    Eigen::Matrix3d R0 = G0.block(0, 0, 3, 3);
    Eigen::Vector3d T0 = G0.block(0, 3, 3, 1);
    Eigen::Matrix3d R1 = G1.block(0, 0, 3, 3);
    Eigen::Vector3d T1 = G1.block(0, 3, 3, 1);

    double dt = (T1 - T0).norm();
    double dr = Eigen::Quaterniond(R1).angularDistance(Eigen::Quaterniond(R0));
    return Eigen::Vector2d(dt, dr);
}

std::vector<Eigen::Vector3d> voxel_filtering(std::vector<Eigen::Vector3d> &src, double voxel_size)
{
    // get all points and sampling
    const int64_t p1 = 73856093;
    const int64_t p2 = 19349669;
    const int64_t p3 = 83492791;

    std::unordered_map<int64_t, uint8_t> hash_map;

    std::vector<Eigen::Vector3d> res;
    for(size_t p = 0; p < src.size(); p++)
    {
        Eigen::Vector3d P = src[p];

        int64_t x = std::floor(P[0]/voxel_size);
        int64_t y = std::floor(P[1]/voxel_size);
        int64_t z = std::floor(P[2]/voxel_size);
        int64_t key = x*p1 ^ y*p2 ^ z*p3; // unlimited bucket size
        if(hash_map.find(key) == hash_map.end())
        {
            hash_map[key] = 1;
            res.push_back(P);
        }
    }

    return res;
}

std::vector<PT_XYZR> voxel_filtering(std::vector<PT_XYZR> &src, double voxel_size)
{
    // get all points and sampling
    const int64_t p1 = 73856093;
    const int64_t p2 = 19349669;
    const int64_t p3 = 83492791;

    std::unordered_map<int64_t, uint8_t> hash_map;

    std::vector<PT_XYZR> res;
    for(size_t p = 0; p < src.size(); p++)
    {
        int64_t x = std::floor(src[p].x/voxel_size);
        int64_t y = std::floor(src[p].y/voxel_size);
        int64_t z = std::floor(src[p].z/voxel_size);
        int64_t key = x*p1 ^ y*p2 ^ z*p3; // unlimited bucket size
        if(hash_map.find(key) == hash_map.end())
        {
            hash_map[key] = 1;
            res.push_back(src[p]);
        }
    }

    return res;
}

bool parse_info(const QString& info, const QString& info_key, NODE_INFO& result)
{
    QStringList lines = info.split("\n");
    for(int i = 0; i < lines.size(); i++)
    {
        QStringList info_parts = lines[i].split(",");
        if(info_parts.size() > 0 && info_parts[0] == info_key)
        {
            if(info_key == "PTZ" && info_parts.size() == 4)
            {
                result.ptz[0] = info_parts[1].toDouble(); // pan
                result.ptz[1] = info_parts[2].toDouble(); // tilt
                result.ptz[2] = info_parts[3].toDouble(); // zoom
                return true;
            }
            else if(info_key == "POSE" && info_parts.size() == 4)
            {
                result.rpy[0] = info_parts[1].toDouble(); // roll
                result.rpy[1] = info_parts[2].toDouble(); // pitch
                result.rpy[2] = info_parts[3].toDouble(); // yaw
                return true;
            }
            else if(info_key == "SIZE" && info_parts.size() == 4)
            {
                result.sz[0] = info_parts[1].toDouble(); // size x
                result.sz[1] = info_parts[2].toDouble(); // size y
                result.sz[2] = info_parts[3].toDouble(); // size z
                return true;
            }
        }
    }
    return false;
}

QJsonArray pose_to_array(Eigen::Vector3d pose)
{
    QJsonArray res;
    res.append(pose[0]);
    res.append(pose[1]);
    res.append(pose[2]);
    return res;
}

Eigen::Vector3d array_to_pose(QJsonArray arr)
{
    Eigen::Vector3d res;
    res[0] = arr[0].toDouble();
    res[1] = arr[1].toDouble();
    res[2] = arr[2].toDouble();
    return res;
}

QJsonArray links_to_array(std::vector<QString> links)
{
    QJsonArray res;
    for(size_t p = 0; p < links.size(); p++)
    {
        res.append(links[p]);
    }
    return res;
}

std::vector<QString> array_to_links(QJsonArray arr)
{
    std::vector<QString> res;
    for(int p = 0; p < arr.size(); p++)
    {
        res.push_back(arr[p].toString());
    }
    return res;
}

CAM_INTRINSIC string_to_intrinsic(QString str)
{
    // w, h, fx, fy, cx, cy, k1, k2, p1, p2
    QStringList str_list = str.split(",");
    if(str_list.size() != 10)
    {
        return CAM_INTRINSIC();
    }

    CAM_INTRINSIC res;
    res.w = str_list[0].toDouble();
    res.h = str_list[1].toDouble();
    res.fx = str_list[2].toDouble();
    res.fy = str_list[3].toDouble();
    res.cx = str_list[4].toDouble();
    res.cy = str_list[5].toDouble();
    res.k1 = str_list[6].toDouble();
    res.k2 = str_list[7].toDouble();
    res.p1 = str_list[8].toDouble();
    res.p2 = str_list[9].toDouble();
    return res;
}

void precise_sleep(double seconds)
{
    if(seconds <= 0)
    {
        return;
    }

    int fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (fd == -1)
    {
        perror("timerfd_create");
        return;
    }

    struct itimerspec ts;
    ts.it_value.tv_sec = static_cast<time_t>(seconds);
    ts.it_value.tv_nsec = static_cast<long>((seconds - ts.it_value.tv_sec) * 1e9);
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 0;

    if (timerfd_settime(fd, 0, &ts, nullptr) == -1)
    {
        perror("timerfd_settime");
        close(fd);
        return;
    }

    uint64_t expirations;
    read(fd, &expirations, sizeof(expirations));
    close(fd);
}

void remove_duplicates_nodes(std::vector<QString>& src)
{
    if(src.size() < 2)
    {
        return;
    }

    std::unordered_set<QString> seen;
    std::vector<QString> unique_list;
    for(const QString& str : src)
    {
        if(seen.insert(str).second) // insert가 성공하면 true
        {
            unique_list.push_back(str);
        }
    }
    src = unique_list;
}

std::vector<Eigen::Vector3d> sampling_line(Eigen::Vector3d P0, Eigen::Vector3d P1, double step)
{
    double d = (P1-P0).norm();
    Eigen::Vector3d dir = (P1-P0).normalized();

    std::vector<Eigen::Vector3d> res;
    res.push_back(P0);
    for(double t = step; t < d; t+=step)
    {
        Eigen::Vector3d P = P0 + t*dir;
        res.push_back(P);
    }
    res.push_back(P1);

    return res;
}

#endif // UTILS_CPP
