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

Eigen::Matrix4d intp_tf(double alpha, Eigen::Matrix4d tf0, Eigen::Matrix4d tf1)
{
    alpha = saturation(alpha, 0, 1.0);
    return Sophus::interpolate<Sophus::SE3d>(Sophus::SE3d(tf0), Sophus::SE3d(tf1), alpha).matrix();
}

void refine_pose(Eigen::Matrix4d& G)
{
    Eigen::Matrix3d R = G.block(0,0,3,3);
    G.block(0,0,3,3) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
}

void refine_zero(Eigen::Matrix4d& G)
{
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            if(std::abs(G(i,j)) < 1e-6)
            {
                G(i,j) = 0;
            }
        }
    }
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

double sgn(double val)
{
    if(val >= 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
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

std::pair<Eigen::Vector3d, double> calc_seg_pt_dist(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P)
{
    Eigen::Vector3d v = P1 - P0;
    Eigen::Vector3d w = P - P0;

    double c1 = w.dot(v);
    double c2 = v.dot(v);

    if (c1 <= 0)
    {
        return std::make_pair(P0, (P - P0).norm());
    }
    else if (c2 <= c1)
    {
        return std::make_pair(P1, (P - P1).norm());
    }
    else
    {
        double b = c1 / c2;
        Eigen::Vector3d Pb = P0 + b * v;
        return std::make_pair(Pb, (P - Pb).norm());
    }
}

bool calc_seg_sphere_intersection(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P, double r, Eigen::Vector3d& intersection)
{
    Eigen::Vector3d d = P1 - P0;
    Eigen::Vector3d v = P0 - P;
    double a = d.dot(d);
    double b = 2.0 * d.dot(v);
    double c = v.dot(v) - r * r;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        return false;
    }

    double t1 = (-b - std::sqrt(discriminant)) / (2 * a);
    double t2 = (-b + std::sqrt(discriminant)) / (2 * a);

    bool intersect1 = (t1 >= 0 && t1 <= 1);
    bool intersect2 = (t2 >= 0 && t2 <= 1);

    if (intersect1 && intersect2)
    {
        Eigen::Vector3d point1 = P0 + t1 * d;
        Eigen::Vector3d point2 = P0 + t2 * d;
        if ((point1 - P1).norm() < (point2 - P1).norm())
        {
            intersection = point1;
        }
        else
        {
            intersection = point2;
        }

        return true;
    }

    if(intersect1)
    {
        intersection = P0 + t1 * d;
        return true;
    }

    if(intersect2)
    {
        intersection = P0 + t2 * d;
        return true;
    }

    return false;
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

double calc_curvature(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2)
{
    double dx1 = P1.x() - P0.x();
    double dy1 = P1.y() - P0.y();
    double dx2 = P2.x() - P1.x();
    double dy2 = P2.y() - P1.y();

    double area = std::abs(dx1 * dy2 - dx2 * dy1);
    double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    double len3 = std::sqrt((P2.x() - P0.x()) * (P2.x() - P0.x()) + (P2.y() - P0.y()) * (P2.y() - P0.y()));

    if(area == 0 || len1 == 0 || len2 == 0 || len3 == 0)
    {
        return 0.0;
    }

    return std::abs(2.0 * area / (len1 * len2 * len3));
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

double calc_length(std::vector<Eigen::Vector3d>& src)
{
    if(src.size() >= 2)
    {
        double sum_d = 0;
        for(size_t p = 0; p < src.size()-1; p++)
        {
            sum_d += calc_dist_2d(src[p+1]-src[p]);
        }
        return sum_d;
    }
    else
    {
        return 0;
    }
}

double calc_min_dist(std::vector<Eigen::Vector3d>& src, Eigen::Vector3d pos)
{
    if(src.size() >= 2)
    {
        double min_dist = 99999999;
        for(size_t p = 0; p < src.size()-1; p++)
        {
            double dist = calc_seg_dist(src[p], src[p+1], pos);
            if(dist < min_dist)
            {
                min_dist = dist;
            }
        }
        return min_dist;
    }
    else
    {
        double dist = calc_dist_2d(src.front()-pos);
        return dist;
    }
}

double calc_similarity(const std::vector<Eigen::Vector3d> &src0, const std::vector<Eigen::Vector3d> &src1)
{
    int n = src0.size();
    int m = src1.size();
    if(n == 0 || m == 0)
    {
        return 0;
    }

    std::vector<std::vector<double>> dtw(n + 1, std::vector<double>(m + 1, std::numeric_limits<double>::infinity()));
    dtw[0][0] = 0.0;

    for (int i = 1; i <= n; ++i)
    {
        for (int j = 1; j <= m; ++j)
        {
            double cost = (src0[i - 1] - src1[j - 1]).norm();
            dtw[i][j] = cost + std::min({dtw[i - 1][j], dtw[i][j - 1], dtw[i - 1][j - 1]});
        }
    }

    double dtw_distance = dtw[n][m];

    double max_length = std::max(n, m);
    double similarity = std::exp(-dtw_distance / max_length);
    return similarity;
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

double check_lr(double ref_x, double ref_y, double ref_yaw, double x, double y)
{
    double x1 = ref_x;
    double y1 = ref_y;
    double x2 = ref_x + 1.0 * std::cos(ref_yaw);
    double y2 = ref_y + 1.0 * std::sin(ref_yaw);
    double vx = x2 - x1;
    double vy = y2 - y1;
    double wx = x - x1;
    double wy = y - y1;
    double s = vx * wy - vy * wx; // s>0 : 차량이 경로의 왼쪽, s<0 : 차량이 경로의 오른쪽
    return s;
}

Eigen::Matrix4d reversed_Lidar(Eigen::Matrix4d tf)
{
    Eigen::Matrix4d reverse;
    reverse.setIdentity();

    reverse(1,1) = -1;
    reverse(2,2) = -1;

    Eigen::Matrix4d ref = tf*reverse;
    return ref;
}

Eigen::Matrix4d elim_rx_ry(Eigen::Matrix4d tf)
{
    Eigen::Matrix3d rotation_matrix = tf.block<3, 3>(0, 0);
    double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

    Eigen::Matrix3d yaw_matrix;
    yaw_matrix << std::cos(yaw), -std::sin(yaw), 0.0,
                  std::sin(yaw), std::cos(yaw), 0.0,
                  0.0, 0.0, 1.0;

    Eigen::Matrix4d new_tf = Eigen::Matrix4d::Identity();
    new_tf.block<3, 3>(0, 0) = yaw_matrix;
    new_tf.block<3, 1>(0, 3) = tf.block<3, 1>(0, 3);
    return new_tf;
}


#endif // UTILS_CPP
