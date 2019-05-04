/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <mpepc/training/agent_state.h>
#include <mpepc/simulator/dynamic_object_trajectory.h>
#include <hssh/local_topological/area.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/area_visitor.h>
#include <hssh/local_topological/areas/serialization.h>
#include <math/clustering.h>
#include <utils/histogram.h>
#include <core/matrix.h>
#include <utils/serialized_file_io.h>
#include <gnuplot-iostream.h>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <iostream>


using namespace vulcan;
using namespace vulcan::hssh;
using namespace vulcan::mpepc;


class MotionVisitor : public LocalAreaVisitor
{
public:

    MotionVisitor(const MotionObs& motions);

    AreaType type(void) const { return type_; }
    double eigRatio(void) const { return eigvals_[0] / eigvals_[1]; }

    MotionObs samples(void) const { return motions_; }

    bool isValidExample(void) const { return (eigvals_[1] > 0) && (motions_.size() > 2); }

    // LocalAreaVisitor interface
    void visitDestination(const LocalDestination& destination) override;
    void visitDecisionPoint(const LocalDecisionPoint& decision) override;
    void visitPathSegment(const LocalPathSegment& path) override;

private:

    AreaType type_ = AreaType::area;

    Matrix eigvecs_;
    Vector eigvals_;

    double maxWidth_;   // maximum width of a gateway in the area

    MotionObs motions_;

    Vector computeEigs(void);
    void mapMotionsViaPCA(const Vector& mean);
    void normalizeLateralDists(const LocalArea& area);
};


void cluster_eig_ratios(const std::vector<MotionVisitor>& motions);
void cluster_lateral_dist(const std::vector<MotionVisitor>& motions);
void cluster_longitudinal_dist(const std::vector<MotionVisitor>& motions);
void cluster_longitudinal_vels(const std::vector<MotionVisitor>& motions);
void plot_1d_clusters(const math::clustering_result_t& clusters,
                      const std::vector<double>& values,
                      const std::string& name);
void plot_2d_clusters(const math::clustering_result_t& clusters,
                      const std::vector<Point<float>>& values,
                      const std::string& name);


int main(int argc, char** argv)
{
    if((argc < 3) || (argc % 2 == 0))
    {
        std::cout << "Usage: social_norms_clustering 'ltm' 'ltm1' ...\n";
        return 1;
    }

    std::vector<MotionVisitor> motions;

    for (int n = 1; n < argc; ++n)
    {
        LocalTopoMap ltm;
        if(!utils::load_serializable_from_file(argv[n], ltm))
        {
            std::cerr << "ERROR: Failed to load LTM from " << argv[n] << '\n';
            return 1;
        }

        MotionObsMap observations = load_motion_observations(argv[n]);
        if(observations.empty())
        {
            std::cerr << "ERROR: Failed to load any data from " << argv[n] << '\n';
            return 1;
        }

        // Process each batch of observations
        for(auto& areaToObs : observations)
        {
            auto area = ltm.areaWithId(areaToObs.first);

            if(area)
            {
                MotionVisitor visitor(areaToObs.second);
                area->accept(visitor);

                // Not every area ends up with enough data to analyze
                if(visitor.isValidExample())
                {
                    motions.push_back(visitor);
                }
            }
        }
    }

    cluster_eig_ratios(motions);
    cluster_lateral_dist(motions);
    cluster_longitudinal_dist(motions);
    cluster_longitudinal_vels(motions);

    return 0;
}


void cluster_eig_ratios(const std::vector<MotionVisitor>& motions)
{
    std::vector<double> ratios(motions.size());
    std::transform(motions.begin(), motions.end(), ratios.begin(), [](auto& m) { return m.eigRatio(); });

    auto clusters = math::em_1d_linear(ratios.begin(), ratios.end(), 5);

    std::cout << "Eig Ratio Clusters:\n";
    for(int n = 0; n < clusters.numClusters; ++n)
    {
        std::cout << n << ": size: " << clusters.clusterSizes[n];

        double mean = 0.0;
        int numPaths = 0;
        int numPlaces = 0;

        for(std::size_t i = 0; i < clusters.assignedCluster.size(); ++i)
        {
            if(clusters.assignedCluster[i] == n)
            {
                mean += ratios[i];

                if(motions[i].type() == AreaType::path_segment)
                {
                    ++numPaths;
                }
                else
                {
                    ++numPlaces;
                }
            }
        }

        mean /= clusters.clusterSizes[n];

        std::cout << " mean: " << mean << " paths: " << numPaths << " places:" << numPlaces << '\n';
    }
}


void cluster_lateral_dist(const std::vector<MotionVisitor>& motions)
{
    std::vector<double> latDists;

//     std::vector<MotionVisitor> paths;
//     std::copy_if(motions.begin(), motions.end(), std::back_inserter(paths), [](const auto& m) {
//         return m.type() == AreaType::path_segment;
//     });
//
//     auto maxSampleIt = std::max_element(paths.begin(), paths.end(), [](const auto& lhs, const auto& rhs) {
//         return lhs.samples().size() < rhs.samples().size();
//     });

    for(auto& m : motions)
    {
        if(m.type() == AreaType::path_segment)
        {
            for(auto& sample : m.samples())
            {
                latDists.push_back(sample.y);
            }
        }
    }

    auto clusters = math::em_1d_linear(latDists.begin(), latDists.end(), 10, 100);

    std::cout << "Lateral Distance Clusters:\n";
    for(int n = 0; n < clusters.numClusters; ++n)
    {
        std::cout << n << ": size: " << clusters.clusterSizes[n];

        double mean = 0.0;

        for(std::size_t i = 0; i < clusters.assignedCluster.size(); ++i)
        {
            if(clusters.assignedCluster[i] == n)
            {
                mean += latDists[i];
            }
        }

        mean /= clusters.clusterSizes[n];

        std::cout << " mean: " << mean << '\n';
    }

    plot_1d_clusters(clusters, latDists, "Lateral Distance");
}


void cluster_longitudinal_dist(const std::vector<MotionVisitor>& motions)
{
    std::vector<double> longDists;

    for(auto& m : motions)
    {
        if(m.type() == AreaType::path_segment)
        {
            for(auto& sample : m.samples())
            {
                longDists.push_back(sample.xVel);
            }
        }
    }

    auto clusters = math::em_1d_linear(longDists.begin(), longDists.end(), 5, 100);

    std::cout << "Longitudinal Distance Clusters:\n";
    for(int n = 0; n < clusters.numClusters; ++n)
    {
        std::cout << n << ": size: " << clusters.clusterSizes[n];

        double mean = 0.0;

        for(std::size_t i = 0; i < clusters.assignedCluster.size(); ++i)
        {
            if(clusters.assignedCluster[i] == n)
            {
                mean += longDists[i];
            }
        }

        mean /= clusters.clusterSizes[n];

        std::cout << " mean: " << mean << '\n';
    }

    plot_1d_clusters(clusters, longDists, "Longitudinal Distance");
}


void cluster_longitudinal_vels(const std::vector<MotionVisitor>& motions)
{
    std::vector<Point<float>> longVels;
//
//     std::vector<MotionVisitor> paths;
//     std::copy_if(motions.begin(), motions.end(), std::back_inserter(paths), [](const auto& m) {
//         return m.type() == AreaType::path_segment;
//     });
//
//     auto maxSampleIt = std::max_element(paths.begin(), paths.end(), [](const auto& lhs, const auto& rhs) {
//         return lhs.samples().size() < rhs.samples().size();
//     });
//
//     for(auto& sample : maxSampleIt->samples())
//     {
//         longVels.emplace_back(sample.xVel, sample.normDist);
//     }

    for(auto& m : motions)
    {
        if(m.type() == AreaType::path_segment)
        {
            for(auto& sample : m.samples())
            {
                longVels.emplace_back(sample.xVel, sample.normDist);
            }
        }
    }

    auto clusters = math::em_2d_linear(longVels.begin(), longVels.end(), 8, 1000);

    std::cout << "Longitudinal Velocity Clusters:\n";
    for(int n = 0; n < clusters.numClusters; ++n)
    {
        std::cout << n << ": size: " << clusters.clusterSizes[n];

        Point<float> mean;

        for(std::size_t i = 0; i < clusters.assignedCluster.size(); ++i)
        {
            if(clusters.assignedCluster[i] == n)
            {
                mean += longVels[i];
            }
        }

        mean.x /= clusters.clusterSizes[n];
        mean.y /= clusters.clusterSizes[n];

        std::cout << " mean: " << mean << '\n';
    }

    plot_2d_clusters(clusters, longVels, "Velocity vs. Lateral Distance");
}


void plot_1d_clusters(const math::clustering_result_t& clusters,
                      const std::vector<double>& values,
                      const std::string& name)
{
    // Assign values to their respective clusters
    std::vector<std::vector<double>> clusterValues(clusters.numClusters);
    for(std::size_t n = 0; n < values.size(); ++n)
    {
        clusterValues[clusters.assignedCluster[n]].push_back(values[n]);
    }

    Gnuplot plot;

    plot << "set title '" << name << "'\n";
    plot << "plot ";

    for(int n = 0; n < clusters.numClusters; ++n)
    {
        plot << "'-' using 1:0 title '" << n << "'";

        if(n != clusters.numClusters - 1)
        {
            plot << ',';
        }
    }

    plot << '\n';

    for(int n = 0; n < clusters.numClusters; ++n)
    {
        std::sort(clusterValues[n].begin(), clusterValues[n].end());
        plot.send1d(clusterValues[n]);
    }
}


void plot_2d_clusters(const math::clustering_result_t& clusters,
                      const std::vector<Point<float>>& values,
                      const std::string& name)
{
    // Assign values to their respective clusters
    std::vector<std::vector<boost::tuple<float, float>>> clusterValues(clusters.numClusters);
    for(std::size_t n = 0; n < values.size(); ++n)
    {
        clusterValues[clusters.assignedCluster[n]].emplace_back(values[n].x, values[n].y);
    }

    Gnuplot plot;

    plot << "set title '" << name << "'\n";
    plot << "plot ";

    for(int n = 0; n < clusters.numClusters; ++n)
    {
        plot << "'-' using 1:2 title '" << n << "'";

        if(n != clusters.numClusters - 1)
        {
            plot << ',';
        }
    }

    plot << '\n';

    for(int n = 0; n < clusters.numClusters; ++n)
    {
        plot.send1d(clusterValues[n]);
    }
}

/////////////////   MotionVisitor implementation   ////////////////////

MotionVisitor::MotionVisitor(const MotionObs& motions)
: motions_(motions)
{
    assert(!motions.empty());

    auto mean = computeEigs();
    mapMotionsViaPCA(mean);
}


void MotionVisitor::visitDestination(const LocalDestination& destination)
{
    type_ = AreaType::destination;
}


void MotionVisitor::visitDecisionPoint(const LocalDecisionPoint& decision)
{
    type_ = AreaType::decision_point;

    std::cout << "\nDecision eig ratio: " << (eigvals_[0] / eigvals_[1]) << '\n';
}


void MotionVisitor::visitPathSegment(const LocalPathSegment& path)
{
    type_ = AreaType::path_segment;

    Vector pathDir(2);
    pathDir[0] = path.plusTransition().target().x - path.minusTransition().target().x;
    pathDir[1] = path.plusTransition().target().y - path.minusTransition().target().y;

    pathDir /= arma::norm(pathDir);

    normalizeLateralDists(path);

    std::cout << "\nPath: " << path.description() << '\n';
    std::cout << "Path vec:\n" << pathDir << " Vel vec:\n" << eigvecs_.col(1);
    std::cout << "Dot product path axis: "
        << arma::dot(pathDir, eigvecs_.col(1)) << '\n';
    std::cout << "Path eig ratio: " << (eigvals_[0] / eigvals_[1]) << '\n';
}


Vector MotionVisitor::computeEigs(void)
{
    Vector mean(4);
    mean.zeros();
    for(auto& obs : motions_)
    {
        mean[0] += obs.x;
        mean[1] += obs.y;
        mean[2] += obs.xVel;
        mean[3] += obs.yVel;
    }

    mean /= motions_.size();

    Matrix cov(2, 2);
    cov.zeros();

    for(auto& obs : motions_)
    {
        Vector obsVec(2);
        obsVec[0] = obs.xVel;
        obsVec[1] = obs.yVel;

        Vector diff = obsVec - mean.rows(2, 3);
        cov += diff * arma::trans(diff);
    }

    arma::eig_sym(eigvals_, eigvecs_, cov);

    return mean;
}


void MotionVisitor::mapMotionsViaPCA(const Vector& mean)
{
    double orientation = std::atan2(eigvecs_(1, 1), eigvecs_(0, 1));
    orientation = wrap_to_pi_2(orientation);

    Line<double> xAxis(mean[0], mean[1], mean[0] + std::cos(orientation), mean[1] + std::sin(orientation));
    Line<double> yAxis(mean[0],
                             mean[1],
                             mean[0] + std::cos(orientation + M_PI_2),
                             mean[1] + std::sin(orientation + M_PI_2));

    for(auto& m : motions_)
    {
        Point<double> pos(m.x, m.y);

        m.x = distance_to_line(pos, yAxis);
        m.y = distance_to_line(pos, xAxis);

        if(left_of_line(yAxis, pos))
        {
            m.x *= -1;
        }

        if(!left_of_line(xAxis, pos))
        {
            m.y *= -1;
        }

        Point<double> vel(m.xVel, m.yVel);
        vel = rotate(vel, orientation);
        m.xVel = vel.x;
        m.yVel = vel.y;
    }
}


void MotionVisitor::normalizeLateralDists(const LocalArea& area)
{
    auto gwys = area.gateways();
    auto maxGwyIt = std::max_element(gwys.begin(), gwys.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.length() < rhs.length();
    });

    if(maxGwyIt == gwys.end())
    {
        return;
    }

    maxWidth_ = maxGwyIt->length();

    for(auto& m : motions_)
    {
        m.y /= maxWidth_;
    }
}
