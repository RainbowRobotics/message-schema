#ifndef MY_UTILS_H
#define MY_UTILS_H

#include "global_defines.h"

extern cv::Vec3b colors[10];
extern QString AUTO_FSM_STATE_STR[7];

bool ping(std::string ip_address);

extern double st_time_for_get_time;
double get_time0();
double get_time();
QString get_time_str();

Eigen::Matrix4d ZYX_to_TF(double tx, double ty, double tz, double rx, double ry, double rz);
Eigen::Matrix4d ZYX_to_TF(QString str);
Eigen::Matrix4d ZYX_to_TF(Sophus::Vector6d zyx);
Sophus::Vector6d TF_to_ZYX(Eigen::Matrix4d tf);

Eigen::Vector3d TF_to_se2(Eigen::Matrix4d tf);
Eigen::Matrix4d se2_to_TF(Eigen::Vector3d xi);

Eigen::Matrix4d se3_to_TF(Sophus::Vector6d xi);
Sophus::Vector6d TF_to_se3(Eigen::Matrix4d tf);

Eigen::Matrix4d string_to_TF(QString str);
QString TF_to_string(Eigen::Matrix4d TF);

Eigen::Matrix4d intp_tf(double alpha, Eigen::Matrix4d tf0, Eigen::Matrix4d tf1);
std::vector<Eigen::Matrix4d> intp_tf(Eigen::Matrix4d tf0, Eigen::Matrix4d tf1, double dist_step, double th_step);
std::vector<Eigen::Vector3d> intp_pts(Eigen::Vector3d P0, Eigen::Vector3d P1, double step);

void refine_pose(Eigen::Matrix4d& G);
bool compare_view_vector(Eigen::Vector3d V0, const Eigen::Vector3d V1, double threshold);

double calc_seg_dist(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P);
bool check_point_on_segment(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P);

double sgn(double val);
int saturation(int val, int min, int max);
double saturation(double val, double min, double max);
double calc_motion_time(double _s, double _v0, double _v1, double _acc);
double calc_dist_2d(Eigen::Vector3d P);
double calc_cte(std::vector<Eigen::Matrix4d>& src, Eigen::Vector3d pos);
double calc_dth(const Eigen::Matrix4d& G0, const Eigen::Matrix4d& G1);

std::vector<cv::Vec2i> line_iterator(cv::Vec2i pt0, cv::Vec2i pt1);
std::vector<cv::Vec2i> circle_iterator(cv::Vec2i pt, int r);
std::vector<cv::Vec2i> filled_circle_iterator(cv::Vec2i pt, int r);

std::vector<Eigen::Vector3d> circle_iterator_3d(Eigen::Vector3d center, double radius);

pcl::PolygonMesh make_donut(double donut_radius, double tube_radius, Eigen::Matrix4d tf, double r, double g, double b, double a=1.0, int num_segments=30);
Eigen::Matrix4d calc_tf(Eigen::Vector3d P0, Eigen::Vector3d P1);
std::vector<Eigen::Matrix4d> calc_path_tf(std::vector<Eigen::Vector3d>& pos);
std::vector<Eigen::Matrix4d> reorientation_path(std::vector<Eigen::Matrix4d>& path);

Eigen::Matrix4d flip_lidar_tf(Eigen::Matrix4d tf);
Eigen::Matrix3d remove_rz(const Eigen::Matrix3d& rotation_matrix);
Eigen::Vector2d dTdR(Eigen::Matrix4d G0, Eigen::Matrix4d G1);

std::vector<Eigen::Vector3d> voxel_filtering(std::vector<Eigen::Vector3d> &src, double voxel_size);

bool parse_info(const QString& info, const QString& info_key, NODE_INFO& result);

QJsonArray pose_to_array(Eigen::Vector3d pose);
Eigen::Vector3d array_to_pose(QJsonArray arr);
QJsonArray links_to_array(std::vector<QString> links);
std::vector<QString> array_to_links(QJsonArray arr);

#endif // MY_UTILS_H
