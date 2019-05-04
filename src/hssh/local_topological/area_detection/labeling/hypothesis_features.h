/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_features.h
* \author   Collin Johnson
*
* Declaration of HypothesisFeatures.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_HYPOTHESIS_FEATURES_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_HYPOTHESIS_FEATURES_H

#include <core/matrix.h>
#include <core/vector.h>
#include <memory>

namespace vulcan
{
namespace utils { class VisibilityGraph; }
namespace hssh
{

class AreaExtent;
class AreaHypothesis;
class VoronoiIsovistField;
class VoronoiSkeletonGrid;

/**
* current_hypothesis_features_version retrieves the current version of the hypothesis features being used in the program.
*/
int current_hypothesis_features_version(void);

/**
* HypothesisFeatures calculates a number of features pertaining to an AreaHypothesis.
*
* The features calculated for the AreaHypothesis are:
*
*   Voronoi-based features:
*       - widthVariation  :    coefficient of variation of cell widths  -- std width / mean width
*       - axisRatio       :    max / (maj + min)
*
*   Ray-based isovist features:
*       - rayCompactnessMean :  avg ray dist / max ray dist
*
*   Polygon-based isovist features:
*       - circularityMean      :    0 = line, 1 = circle, paths are lower, square-ish rooms are higher -- isoperimetric quotient
*       - shapeCompactnessMean :
*       - eccentricityMean     :
*
*   Visibility-based features:
*       - avgPathEdgeCount :  average number of edges in a path between any two nodes in the hypothesis
*
* Exploration is stored separated from the shape-based features because it isn't the same type of thing and
* provides more of a h*igh-level info on how much of the area has been seen, which shouldn't be involved in
* the final classifier
*/
class HypothesisFeatures
{
public:

    using FeatureVector = Vector;
    using IsovistFeatures = Matrix;

    /**
    * ClearCache clears the cache of features. This function should be called at the start of each labeling.
    */
    static void ClearCache(void);

    /**
    * Constructor for HypothesisFeatures.
    *
    * \param    hypothesis          Hypothesis for which the features are being created
    * \param    extent              Extent of the hypothesis
    * \param    grid                Grid in which the hypothesis lives
    * \param    isovistField        Isovist field for the skeleton cells
    * \param    visGraph            Visibility graph for the grid
    * \param    skeletonGraph       Graph (as a VisibilityGraph for feature purposes) of the skeleton
    */
    HypothesisFeatures(const AreaHypothesis&      hypothesis,
                       const AreaExtent&          extent,
                       const VoronoiSkeletonGrid& grid,
                       const VoronoiIsovistField& isovistField,
                       const utils::VisibilityGraph& visGraph,
                       const utils::VisibilityGraph& skeletonGraph);

    /**
    * Constructor for HypothesisFeatures.
    *
    * Create features using already calculated vector. Useful mostly for testing purposes.
    */
    HypothesisFeatures(const Vector& features, double exploration, double axisRatio);

    /**
    * Default constructor for HypothesisFeatures.
    */
    HypothesisFeatures(void);

    // Can default all constructors
    HypothesisFeatures& operator=(HypothesisFeatures&& rhs) = default;
    HypothesisFeatures& operator=(const HypothesisFeatures& rhs) = default;
    HypothesisFeatures(HypothesisFeatures&& rhs) = default;
    HypothesisFeatures(const HypothesisFeatures& rhs) = default;


    /**
    * numFeatures retrieves the number of features currently being used.
    */
    std::size_t numFeatures(void) const { return features_.size(); }

    /**
    * featureName retrieves the name of the feature with the provided index.
    */
    static std::string featureName(int index);
    static std::string isovistFeatureName(int index);

    /**
    * numIsovists retrieves the number of isovists stored in these features.
    */
    std::size_t numIsovists(void) const { return isovistFeatures_.n_cols; }

    /**
     * numIsovistFeatures retrieves the number of isovists stored in these features.
     */
    std::size_t numIsovistFeatures(void) const { return isovistFeatures_.n_rows; }

    /**
    * version retrieves the version number for the features. Every time these features are changed, the version
    * increases by 1. Thus, the version number is closely associated with the number of different attempts at made at
    * finding a good feature set.
    *
    * There are separate version numbers for the area features and individual isovist features, since they are
    * maintained and computed separately.
    */
    int version(void) const { return version_; }
    int isovistVersion(void) const { return isovistVersion_; }

    double axisRatio(void) const { return axisRatio_; }
    double explorationAmount(void) const { return exploration_; }

    double               featureAt    (std::size_t index) const { return features_[index]; }
    const FeatureVector& featureVector(void)              const { return features_;        }

    /**
    * isovistFeatures are stored with one column per isovist. The dimensions are: numIsovistFeatures x numIsovists
    */
    const IsovistFeatures& isovistFeatures(void) const { return isovistFeatures_; }

    FeatureVector::const_iterator begin(void) const { return features_.begin(); }
    FeatureVector::const_iterator end  (void) const { return features_.end();   }

    // I/O operators for HypothesisFeatures
    /**
    * The format for HypothesisFeatures output is:
    *
    *   num_features feature_0 ... feature_n-1 num_isovists num_isovist_features iso_feats_00 iso_feats_01 ... exploration
    */
    friend std::ostream& operator<<(std::ostream& out, const HypothesisFeatures& features);
    friend std::istream& operator>>(std::istream& in,  HypothesisFeatures& features);

private:

    int version_;
    int isovistVersion_;
    FeatureVector features_;
    IsovistFeatures isovistFeatures_;
    double exploration_;
    double axisRatio_;

    void calculateStructureFeatures(const AreaHypothesis& hypothesis, const VoronoiSkeletonGrid& grid);
    void calculateAxisFeatures(const AreaHypothesis& hypothesis, const VoronoiSkeletonGrid& grid);
    void calculateShapeFeatures(const AreaExtent& extent, const VoronoiSkeletonGrid& grid);
    void calculateIsovistFeatures(const AreaHypothesis& hypothesis, const VoronoiIsovistField& isovistField);
    void calculateZernikeFeature(const AreaExtent& extent);
    void calculateVisGraphFeatures(const AreaHypothesis& hypothesis, const utils::VisibilityGraph& visGraph);
    void calculateSkeletonGraphFeatures(const AreaHypothesis& hypothesis, const utils::VisibilityGraph& skeletonGraph);
};

// Equality operators for HypothesisFeatures
bool operator==(const HypothesisFeatures& lhs, const HypothesisFeatures& rhs);
bool operator!=(const HypothesisFeatures& lhs, const HypothesisFeatures& rhs);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_HYPOTHESIS_FEATURES_H
