//
// Created by jy on 25. 12. 28..
//

#ifndef SLAMNAV2_SLAMNAV_MAP_TYPES_H
#define SLAMNAV2_SLAMNAV_MAP_TYPES_H

//#include "slamnav_slam_types.h"

#include <QObject>
#include <QString>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/StageFactory.hpp>

// nanoflann
#include "nanoflann.hpp"


struct PT_XYZR
{
    int k       = 0;       // add cnt of tree
    int k0      = 0;      // original add cnt
    int do_cnt  = 0;  // dynamic object count
    double x    = 0;    // x coordinates
    double y    = 0;    // y coordinates
    double z    = 0;    // z coordinates
    double vx   = 0;   // view vector x
    double vy   = 0;   // view vector y
    double vz   = 0;   // view vector z
    double r    = 0;    // reflect 0~1
};

struct PT_SURFEL
{
    int lb    = 0;   // 0:none, 1:travel
    double x  = 0.0;
    double y  = 0.0;
    double z  = 0.0;
    double r  = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    double nz = 0.0;
};

struct KFRAME
{
    int id = 0;
    Eigen::Matrix4d G;
    Eigen::Matrix4d opt_G;
    std::vector<PT_XYZR> pts;

    KFRAME()
    {
        id = 0;
        G.setIdentity();
        opt_G.setIdentity();
    }

    KFRAME(const KFRAME& p)
    {
        id = p.id;
        pts = p.pts;
        G = p.G;
        opt_G = p.opt_G;
    }

    KFRAME& operator=(const KFRAME& p)
    {
        id = p.id;
        pts = p.pts;
        G = p.G;
        opt_G = p.opt_G;
        return *this;
    }
};

struct XYZR_CLOUD
{
    std::vector<PT_XYZR> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional: computation of bounding box
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

struct XYZ_NODE
{
    std::vector<Eigen::Vector3d> pos;
    std::vector<QString> id;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pos.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return pos[idx][0];
        else if (dim == 1) return pos[idx][1];
        else return pos[idx][2];
    }

    // Optional: computation of bounding box
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, XYZ_NODE>, XYZ_NODE, 3> KD_TREE_NODE;

struct COST_JACOBIAN
{
    double c = 0;
    double w = 0;
    double J[12] = {0,};

    COST_JACOBIAN(){}

    COST_JACOBIAN(const COST_JACOBIAN& p)
    {
        c = p.c;
        w = p.w;
        memcpy(J, p.J, sizeof(double)*12);
    }

    COST_JACOBIAN& operator=(const COST_JACOBIAN& p)
    {
        c = p.c;
        w = p.w;
        memcpy(J, p.J, sizeof(double)*12);
        return *this;
    }
};

// tree typedef
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, XYZR_CLOUD>, XYZR_CLOUD, 3> KD_TREE_XYZR;

struct CLOUD
{
    std::vector<Eigen::Vector3d> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if(dim == 0)
        {
            return pts[idx][0];
        }
        else if(dim == 1)
        {
            return pts[idx][1];
        }
        else
        {
            return pts[idx][2];
        }
    }

    // Optional: computation of bounding box
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, CLOUD>, CLOUD, 3> KD_TREE;

struct PICKING
{
    bool l_drag;
    Eigen::Vector3d l_pt0;
    Eigen::Vector3d l_pt1;
    Eigen::Vector3d l_pose;

    bool r_drag;
    Eigen::Vector3d r_pt0;
    Eigen::Vector3d r_pt1;
    Eigen::Vector3d r_pose;

    // node
    QString pre_node;
    QString cur_node;

    int last_btn = 0; // 0: left, 1: right

    PICKING()
    {
        l_drag = false;
        l_pt0.setZero();
        l_pt1.setZero();
        l_pose.setZero();

        r_drag = false;
        r_pt0.setZero();
        r_pt1.setZero();
        r_pose.setZero();

        pre_node = "";
        cur_node = "";

        last_btn = 0;
    }

    PICKING(const PICKING& p)
    {
        l_drag = p.l_drag;
        l_pt0 = p.l_pt0;
        l_pt1 = p.l_pt1;
        l_pose = p.l_pose;

        r_drag = p.r_drag;
        r_pt0 = p.r_pt0;
        r_pt1 = p.r_pt1;
        r_pose = p.r_pose;

        pre_node = p.pre_node;
        cur_node = p.cur_node;

        last_btn = p.last_btn;
    }

    PICKING& operator=(const PICKING& p)
    {
        l_drag = p.l_drag;
        l_pt0 = p.l_pt0;
        l_pt1 = p.l_pt1;
        l_pose = p.l_pose;

        r_drag = p.r_drag;
        r_pt0 = p.r_pt0;
        r_pt1 = p.r_pt1;
        r_pose = p.r_pose;

        pre_node = p.cur_node;
        cur_node = p.pre_node;

        last_btn = p.last_btn;
        return *this;
    }
};

#endif //SLAMNAV2_SLAMNAV_MAP_TYPES_H