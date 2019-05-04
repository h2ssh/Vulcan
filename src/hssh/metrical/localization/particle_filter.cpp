/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     particle_filter.cpp
* \author   Collin Johnson
*
* Definition of ParticleFilter and create_particle_filter() factory.
*/

#include <hssh/metrical/localization/particle_filter.h>
#include <hssh/metrical/localization/motion_model.h>
#include <hssh/metrical/localization/observation_model.h>
#include <hssh/metrical/localization/particle_sampler.h>
#include <hssh/metrical/localization/sample_set_distribution_calculator.h>
#include <hssh/metrical/localization/particle_filter_utils.h>
#include <hssh/metrical/data.h>
#include <hssh/metrical/occupancy_grid.h>
#include <hssh/metrical/localization/debug_info.h>
#include <laser/laser_scan_lines.h>
#include <core/angle_functions.h>
#include <utils/timestamp.h>
#include <gnuplot-iostream.h>
#include <iostream>
#include <cassert>

#include <future>
#include <thread>


#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>


// #define DEBUG_POSTERIOR_CALCULATION
// #define DEBUG_MEAN
// #define DEBUG_COVARIANCE
// #define DEBUG_UNCERTAINTY
// #define DEBUG_SAMPLING

namespace vulcan
{
namespace hssh
{

void display_posterior_weight_statistics(const std::vector<particle_t>& posterior);


ParticleFilter::ParticleFilter(const particle_filter_params_t&                  params,
                               std::unique_ptr<MotionModel>                     motion,
                               std::unique_ptr<ObservationModel>                observation,
                               std::unique_ptr<ParticleSampler>                 sampler,
                               std::unique_ptr<SampleSetDistributionCalculator> calcuator)
: isInitialUpdate (true)
, kldSampling     (params.kldParams)
, motionModel     (std::move(motion))
, observationModel(std::move(observation))
, sampler         (std::move(sampler))
, calculator      (std::move(calcuator))
, previousTime    (0)
, currentTime     (0)
, params          (params)
{
    deltaOut.open("estimate_diff.log");
}


ParticleFilter::~ParticleFilter(void)
{
}


pose_distribution_t ParticleFilter::initializeFilterAtPose(const pose_t& pose)
{
    // Initialize filter with uniformly weighted particles drawn from pose distribution
    double sampleWeight = 1.0 / params.initialNumSamples;

    currentPose = createDistributionForResetPose(pose);
    currentPose.uncertainty.prepareForSampling();

    prior.samples.clear();

    particle_t newParticle;

    for(std::size_t n = 0; n < params.initialNumSamples; ++n)
    {
        auto sampledPose   = currentPose.uncertainty.sample();
        newParticle.pose   = pose_t(sampledPose(0), sampledPose(1), sampledPose(2));
        newParticle.parent = newParticle.pose;
        newParticle.weight = sampleWeight;

        prior.samples.push_back(newParticle);
    }

    prior.gaussianApproximation = currentPose.uncertainty;
    isInitialUpdate             = true;

    return currentPose;
}


pose_distribution_t ParticleFilter::initializeWithSampleSet(const std::vector<particle_t>& samples)
{
    prior.samples = samples;
    for(auto& s : prior.samples)
    {
        s.weight = 1.0 / prior.samples.size();
    }

    currentPose = calculate_sample_set_distribution(samples);
    prior.gaussianApproximation = currentPose.uncertainty;

    // Ensure the parents are set for all samples
    for(auto& sample : prior.samples)
    {
        sample.parent = sample.pose;
    }

    isInitialUpdate = true;

    return currentPose;
}


pose_distribution_t ParticleFilter::updateFilter(const metric_slam_data_t&     data,
                                                        const OccupancyGrid&          map,
                                                        particle_filter_debug_info_t* debug)
{
    if(debug)
    {
        debug->particles.clear();
        debug->particleScores.clear();
        debug->scan = data.laser;
    }

    if(isInitialUpdate)
    {
        calculateInitialParticleWeights(data.scanLines, map, debug);
        isInitialUpdate = false;
        previousTime    = data.endTime;
    }

    currentTime = data.endTime;

    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = motionModel->calculateUpdateMotion(data, prior.gaussianApproximation, data.endTime-data.startTime);

    if(hasRobotMoved)// || params.shouldLocalizeWhenNoMotionDetected)
    {
        calculateProposalDistribution(motionModel->getMotionDistribution());
        observationModel->initializeModel(data.scanLines, proposalDistribution);
        calculatePosteriorDistribution(data.scanLines, map, debug);

        assert(posterior.samples.size() > 1);

        posterior.gaussianApproximation = calculator->calculateGaussianForSamples(posterior.samples);
        prior = posterior;
    }

    previousTime = currentTime;

    if(debug)
    {
        debug->particles            = posterior.samples;
        debug->proposalDistribution = proposalDistribution;
    }

#ifdef DEBUG_SAMPLING
    display_posterior_weight_statistics(posterior.samples);
#endif

//     double xDiff = posterior.gaussianApproximation[0] - currentPose.x;
//     double yDiff = posterior.gaussianApproximation[1] - currentPose.y;
//     double thetaDiff = angle_diff(posterior.gaussianApproximation[2], currentPose.theta);
//     double trans = std::sqrt(xDiff*xDiff + yDiff*yDiff);
//     double rot = thetaDiff;
//
//     double odomTrans = std::sqrt(std::pow(motionModel->getMotionDistribution()[0], 2.0)
//         + std::pow(motionModel->getMotionDistribution()[1], 2.0));
//     double odomRot = motionModel->getMotionDistribution()[2];
//
//     sumXDiff += xDiff - motionModel->getMotionDistribution()[0];
//     sumYDiff += yDiff - motionModel->getMotionDistribution()[1];
//     sumThetaDiff += angle_diff(thetaDiff, motionModel->getMotionDistribution()[2]);
//     sumTransDiff += trans - odomTrans;
//     sumRotDiff += rot - odomRot;
//     ++numUpdates;
//
//     if(std::abs(rot - odomRot) > M_PI / 18)
//     {
//         assert(!"Orientation jumped!");
//     }
//
//     deltaOut << numUpdates << ' '
//         << (xDiff - motionModel->getMotionDistribution()[0]) << ' '
//         << (yDiff - motionModel->getMotionDistribution()[1]) << ' '
//         << (thetaDiff - motionModel->getMotionDistribution()[2]) << ' '
//         << (trans - odomTrans) << ' '
//         << (rot - odomRot) << ' '
//         << (sumXDiff / numUpdates) << ' '
//         << (sumYDiff / numUpdates) << ' '
//         << (sumThetaDiff / numUpdates) << ' '
//         << (sumTransDiff / numUpdates) << ' '
//         << (sumRotDiff / numUpdates) << '\n';
//
//     std::cout << "Avg diff: (" << (sumXDiff / numUpdates) << ", " << (sumYDiff / numUpdates) << ", "
//         << (sumThetaDiff / numUpdates) << ") Trans:" << (sumTransDiff / numUpdates) << " Rot:"
//         << (sumRotDiff / numUpdates) << " Num updates:" << numUpdates << '\n';

    // If the pose jumped, fall back on odometry, but keep the same uncertainty
//     if(
//         (std::abs(currentPose.x - posterior.gaussianApproximation[0]) > 0.25) ||
//         (std::abs(currentPose.y - posterior.gaussianApproximation[1]) > 0.25) ||
//         (posterior.gaussianApproximation(0, 0) > 0.1) ||
//         (posterior.gaussianApproximation(1, 1) > 0.1))
//     {
//         std::cout << "\n\nERROR: Localization jumped: Cov:" << posterior.gaussianApproximation.getCovariance()
//             << "\n\n\nFalling back on odometry";
//
//         pose_t odomPose = motionModel->movePoseByMean(currentPose.toPose());
//         currentPose.x = odomPose.x;
//         currentPose.y = odomPose.y;
//         currentPose.theta = odomPose.theta;
//         posterior.gaussianApproximation[0] = odomPose.x;
//         posterior.gaussianApproximation[1] = odomPose.y;
//         posterior.gaussianApproximation[2] = odomPose.theta;
//     }
//     else
//     {
        assert(posterior.gaussianApproximation.dimensions() == 3);

        currentPose.x           = posterior.gaussianApproximation[0];
        currentPose.y           = posterior.gaussianApproximation[1];
        currentPose.theta       = posterior.gaussianApproximation[2];
        currentPose.uncertainty = posterior.gaussianApproximation;
//     }

    currentPose.uncertainty = posterior.gaussianApproximation;
    currentPose.timestamp   = data.endTime;

    assert(currentPose.timestamp);

#ifdef DEBUG_UNCERTAINTY
    Matrix covariance = currentPose.uncertainty.getCovariance();

    float stdX     = std::sqrt(covariance(0,0));
    float stdY     = std::sqrt(covariance(1,1));
    float stdTheta = std::sqrt(covariance(2,2));

    std::cout<<"INFO: ParticleFilter: Mean:" << currentPose.toPose() << " Std:("<<stdX<<','<<stdY<<','<<stdTheta<<")\n";
#endif

    return currentPose;
}


void ParticleFilter::changeReferenceFrame(const pose_t& referenceFrame)
{
    for(auto& sample : prior.samples)
    {
        sample.pose = sample.pose.transformToNewFrame(referenceFrame);
    }

    pose_t transformedMean = pose_t(currentPose).transformToNewFrame(referenceFrame);

    currentPose.x     = transformedMean.x;
    currentPose.y     = transformedMean.y;
    currentPose.theta = transformedMean.theta;

    // HACK: Not quite correct because the covariance would need to be transformed by the Jacobian of the rotation matrix
    // but should be fine for just the single update the will happen before the new reference frame has kicked fully into gear
    Vector meanVector = currentPose.uncertainty.getMean();
    meanVector(0) = transformedMean.x;
    meanVector(1) = transformedMean.y;
    meanVector(2) = transformedMean.theta;
    currentPose.uncertainty.setDistributionStatistics(meanVector, currentPose.uncertainty.getCovariance());
}


void ParticleFilter::calculatePosteriorDistribution(const laser::laser_scan_lines_t& scan,
                                                    const OccupancyGrid&             map,
                                                    particle_filter_debug_info_t*    debug)
{
    assert(prior.gaussianApproximation.dimensions() == 3);

    particle_t meanSample(prior.gaussianApproximation[0],
                          prior.gaussianApproximation[1],
                          prior.gaussianApproximation[2]);

    kldSampling.startNewCalculation(meanSample);
    prior.gaussianApproximation.prepareForSampling();

    proposal.samples.clear();
    posterior.samples.clear();

    int samplesDrawn  = 0;
    int samplesToDraw = 0;

    do
    {
        samplesToDraw = std::min(kldSampling.currentNumberOfSamplesNeeded(), params.maxSamplesDrawn) - samplesDrawn;

        sampler->drawSamplesFromPrior(samplesToDraw, prior, proposal.samples);

        for(int n = samplesDrawn; n < samplesDrawn+samplesToDraw; ++n)
        {
            posterior.samples.push_back(processNewSample(proposal.samples[n], scan, map, debug));
            posterior.samples.back().id = n;
        }

        samplesDrawn += samplesToDraw;
    } while(!kldSampling.haveDrawnEnoughSamples(samplesDrawn) && (samplesToDraw > 0));

//     std::vector<particle_t> samples;
//     prior.gaussianApproximation.prepareForSampling();
//
//     int numSamples = params.kldParams.minSamples;
//     const int kMaxThreads = 6;
//     const int kNumThreads = kMaxThreads; //std::min(kMaxThreads, (numSamples / kMaxThreads) + 1);
//
//     int samplesPerThread = numSamples;// / kNumThreads; // Add one to the positions/thread to account for truncating due to integer division
//                                                     // The last thread will take up to 3 fewer isovist calculations
//     numSamples = samplesPerThread * kNumThreads;
//     samples.resize(numSamples);
//     using SampleIter = std::vector<particle_t>::iterator;
//     auto sampleFunc = [&](SampleIter begin, SampleIter end) {
//         std::vector<particle_t> sampled;
//         sampled.reserve(samplesPerThread);
//         sampler->drawSamplesFromPrior(samplesPerThread, prior, sampled);
//         for(int n = 0; n < samplesPerThread; ++n)
//         {
//             *begin = processNewSample(sampled[n], scan, map, nullptr);
//             begin->id = n;
//             ++begin;
//         }
//
//         assert(begin == end);
//     };
//
//     // Launch the threads
//     std::vector<std::future<void>> asyncSamples;
//     for(int n = 0; n < kNumThreads; ++n)
//     {
//         int start = samplesPerThread * n;
//         int end = samplesPerThread * (n+1);
//         assert(end <= numSamples);
//
//         asyncSamples.push_back(std::async(std::launch::async,
//                                         sampleFunc,
//                                         samples.begin()+start,
//                                         samples.begin()+end));
//     }
//
//     // Wait for all the samples to finish
//     for(auto& f : asyncSamples)
//     {
//         f.get();
//     }
//
//     for(auto& sample : samples)
//     {
//         posterior.samples.push_back(sample);
//     }
//
//     std::cout << "Drew " << posterior.samples.size() << " samples.\n";

//     std::vector<double> weights(posterior.samples.size());
//     std::transform(posterior.samples.begin(), posterior.samples.end(), weights.begin(), [](const particle_t& p) {
//         return p.weight;
//     });
//     std::sort(weights.begin(), weights.end(), std::greater<double>());
//
//     static Gnuplot plot;
//     double cutoff = weights.front() * 0.9;
//     auto cutoffIt = std::find_if(weights.begin(), weights.end(), [cutoff](double w) { return w < cutoff; });
//     int numGood = std::distance(weights.begin(), cutoffIt);
//     plot << "unset arrow\n";
//     plot << "set xrange [0:500]\n";
//     plot << "set arrow from " << numGood << ",0 to " << numGood << ',' << cutoff <<" nohead\n";
//     plot << "set arrow from 0," << cutoff << " to " << numGood << ',' << cutoff << " nohead\n";
//     plot << "set xtics (" << numGood << ' ' << numGood << ")\n";
//     plot << "set ytics (" << cutoff << ' ' << cutoff << ")\n";
//     plot << "plot '-' with lines title 'Particle Weights', " << cutoff << " title '" << numGood << "'\n";
// //     plot[scan.scan.laserId] << "plot '-' with lines title 'Dist Weights', '-' with lines title 'Line Weights'\n";
//     plot.send1d(weights);
//     plot[scan.scan.laserId].send1d(lineWeights);
//     plot[scan.scan.laserId].send1d(weights);

    normalize_sample_weights(posterior.samples);

#ifdef DEBUG_SAMPLING
    std::cout<<"INFO: ParticleFilter: Drew "<<samplesDrawn<<" samples\n";
#endif
}


void ParticleFilter::calculateInitialParticleWeights(const laser::laser_scan_lines_t& scan,
                                                     const OccupancyGrid&             map,
                                                     particle_filter_debug_info_t*    debug)
{
    observationModel->initializeModel(scan, currentPose.uncertainty);

    for(auto& sample : prior.samples)
    {
        sample.weight = observationModel->sampleLikelihood(sample, scan, map, debug);
    }

    normalize_sample_weights(prior.samples);
    currentPose = calculate_sample_set_distribution(prior.samples);

    posterior = prior;
}


void ParticleFilter::calculateProposalDistribution(const MultivariateGaussian& motionTransform)
{
    Matrix rotation(3, 3);

    rotation.zeros();
    rotation(0, 0) = std::cos(currentPose.theta);
    rotation(0, 1) = -std::sin(currentPose.theta);
    rotation(1, 0) = std::sin(currentPose.theta);
    rotation(1, 1) = std::cos(currentPose.theta);
    rotation(2, 2) = 1.0;

    Vector proposalMean       = currentPose.uncertainty.getMean() + rotation*motionTransform.getMean();
    Matrix proposalCovariance = currentPose.uncertainty.getCovariance() + arma::trans(rotation)*motionTransform.getCovariance()*rotation;

    proposalMean(2) = wrap_to_pi(proposalMean(2));
    proposalDistribution.setDistributionStatistics(proposalMean, proposalCovariance);
}


particle_t ParticleFilter::processNewSample(const particle_t&                parent,
                                            const laser::laser_scan_lines_t& scan,
                                            const OccupancyGrid&             map,
                                            particle_filter_debug_info_t*    debug)
{
    auto newSample = motionModel->moveSample(parent);

    if(newSample.pose.timestamp == parent.parent.timestamp)
    {
        std::cout<<"Same time for pose and parent!\n";
    }

    newSample.parent.timestamp = previousTime;
    newSample.pose.timestamp   = currentTime;

    // Very unlikely the robot is in an unobserved cell
    newSample.weight = observationModel->sampleLikelihood(newSample, scan, map, debug);
//     kldSampling.addSample(newSample);

#ifdef DEBUG_POSTERIOR_CALCULATION
    std::cout<<"INFO: ParticleFilter: sample:"<<newSample.pose<<" weight:"<<newSample.weight<<'\n';
#endif

    return newSample;
}


pose_distribution_t ParticleFilter::createDistributionForResetPose(const pose_t& pose)
{
    Vector mean(3);
    Matrix cov(3, 3);

    cov.zeros(3, 3);

    mean(0) = pose.x;
    mean(1) = pose.y;
    mean(2) = pose.theta;

    cov(0, 0) = params.positionInitialVariance;
    cov(1, 1) = params.positionInitialVariance;
    cov(2, 2) = params.orientationInitialVariance;

    return MultivariateGaussian(mean, cov);
}


void display_posterior_weight_statistics(const std::vector<particle_t>& posterior)
{
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::mean, tag::variance, tag::max, tag::min, tag::median>> statsAcc;

    for(auto& particle : posterior)
    {
        statsAcc(particle.weight);
    }

    std::cout << "Particle weights stats: Mean:" << mean(statsAcc) << " Var:" << variance(statsAcc)
        << " Min:" << min(statsAcc) << " Max:" << max(statsAcc) << " Med:" << median(statsAcc) << '\n';
}

} // namespace hssh
} // namespace vulcan
