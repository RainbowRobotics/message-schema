#ifndef PGO_H
#define PGO_H

// Eigen
#include <Eigen/Core>

// GTSAM
#include <boost/algorithm/clamp.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

class PGO
{
public:
    PGO();

    void clear();
    void add_node(int id, Eigen::Matrix4d dG, Eigen::Matrix4d G, double err);
    void add_lc(int id0, int id1, Eigen::Matrix4d dG, double err);
    std::vector<Eigen::Matrix4d> get_optimal_poses();

private:
    gtsam::ISAM2 isam;

};

#endif // PGO_H
