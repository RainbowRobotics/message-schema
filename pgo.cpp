#include "pgo.h"

PGO::PGO()
{
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.1;
    isam_params.relinearizeSkip = 1;

    isam = gtsam::ISAM2(isam_params);
}

void PGO::add_node(int id, Eigen::Matrix4d dG, Eigen::Matrix4d G, double err)
{
    if(id == 0)
    {
        // set first
        Eigen::Matrix4d origin = G;

        // set graph
        gtsam::Pose3 prior(origin);
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_estimate;

        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.0001)).finished());
        graph.addPrior(0, prior, prior_noise);

        // set initial estimates
        initial_estimate.insert(0, gtsam::Pose3(origin));

        // set current graph and initial estimates
        isam.update(graph, initial_estimate);
        isam.update();
    }
    else
    {
        // id
        int id0 = id-1;
        int id1 = id;

        // set odometry edge
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_estimate;

        auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(err), gtsam::Vector3::Constant(err)).finished());
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(id0, id1, gtsam::Pose3(dG), noise);

        // set initial estimates
        initial_estimate.insert(id1, gtsam::Pose3(G));

        // set current graph and initial estimates
        isam.update(graph, initial_estimate);
        isam.update();
    }
}

void PGO::add_lc(int id0, int id1, Eigen::Matrix4d dG, double err)
{
    gtsam::NonlinearFactorGraph graph;
    auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(err), gtsam::Vector3::Constant(err)).finished());
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(id0, id1, gtsam::Pose3(dG), noise);

    isam.update(graph);
    isam.update();
}

std::vector<Eigen::Matrix4d> PGO::get_optimal_poses()
{
    std::vector<Eigen::Matrix4d> res;

    gtsam::Values poses = isam.calculateBestEstimate();
    for(size_t p = 0; p < poses.size(); p++)
    {
        res.push_back(poses.at<gtsam::Pose3>(p).matrix());
    }

    return res;
}


