/**
 * BaroFusion — env-gated barometric-altitude fusion for the inertial bundle adjustments.
 *
 * ORB_BARO_CSV=<path>   ref_baro.csv ("#timestamp [ns],alt_m", rebased ns matching image stamps)
 * ORB_BARO_SIGMA=<m>    1-sigma of one baro altitude constraint (default 1.0 m)
 *
 * After IMU initialization the world frame is gravity-aligned (+z up, ApplyScaledRotation), so the
 * barometric altitude constrains the body z directly. Because later ScaleRefinement /
 * ApplyScaledRotation calls can re-rotate/rescale the map, no global z<->baro datum is stored:
 * each optimization window recomputes its own datum (median of z_kf - baro(t_kf) over the window's
 * keyframes), so the edges only suppress RELATIVE vertical drift inside the window and are
 * gauge-safe under global similarity updates.
 *
 * EdgeBaroZ residual: r = twb.z - (baro(t) + datum).  VertexPose update is body-frame
 * (twb += Rwb*ut, G2oTypes.cc), hence d r / d ut = e_z^T Rwb = Rwb.row(2) and d r / d ur = 0.
 */

#ifndef BAROFUSION_H
#define BAROFUSION_H

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "G2oTypes.h"

namespace ORB_SLAM3
{

class BaroFusion
{
public:
    static BaroFusion& Instance()
    {
        static BaroFusion instance;
        return instance;
    }

    bool Enabled() const { return !mTimesSec.empty(); }

    double Sigma() const { return mSigma; }

    double Gate() const { return mGate; }

    /** Linear interpolation of the barometric altitude at t [s]; false outside the record. */
    bool AltitudeAt(const double timeSec, double& altitude) const
    {
        if (mTimesSec.empty() || timeSec < mTimesSec.front() - 0.5 || timeSec > mTimesSec.back() + 0.5)
        {
            return false;
        }
        const std::vector<double>::const_iterator upper =
            std::lower_bound(mTimesSec.begin(), mTimesSec.end(), timeSec);
        if (upper == mTimesSec.begin())
        {
            altitude = mAltitudes.front();
            return true;
        }
        if (upper == mTimesSec.end())
        {
            altitude = mAltitudes.back();
            return true;
        }
        const size_t hi = upper - mTimesSec.begin();
        const size_t lo = hi - 1;
        const double span = mTimesSec[hi] - mTimesSec[lo];
        const double weight = span > 0.0 ? (timeSec - mTimesSec[lo]) / span : 0.0;
        altitude = (1.0 - weight) * mAltitudes[lo] + weight * mAltitudes[hi];
        return true;
    }

    /** Window datum: median of (z_kf - baro_kf). Gauge-safe: recomputed every optimization. */
    static double WindowDatum(std::vector<double>& offsets)
    {
        std::vector<double>::iterator middle = offsets.begin() + offsets.size() / 2;
        std::nth_element(offsets.begin(), middle, offsets.end());
        return *middle;
    }


private:
    BaroFusion() : mSigma(1.0), mGate(15.0)
    {
        const char* csvPath = getenv("ORB_BARO_CSV");
        if (!csvPath)
        {
            return;
        }
        const char* sigmaEnv = getenv("ORB_BARO_SIGMA");
        if (sigmaEnv)
        {
            mSigma = atof(sigmaEnv);
        }
        const char* gateEnv = getenv("ORB_BARO_GATE");
        if (gateEnv)
        {
            mGate = atof(gateEnv);
        }
        std::ifstream file(csvPath);
        std::string line;
        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }
            const size_t comma = line.find(',');
            if (comma == std::string::npos)
            {
                continue;
            }
            mTimesSec.push_back(std::stod(line.substr(0, comma)) * 1e-9);
            mAltitudes.push_back(std::stod(line.substr(comma + 1)));
        }
        std::cout << "BaroFusion: " << mTimesSec.size() << " baro samples from " << csvPath
                  << " sigma " << mSigma << " m" << std::endl;
    }

    std::vector<double> mTimesSec;
    std::vector<double> mAltitudes;
    double mSigma;
    double mGate;
};

/** Unary barometric-altitude constraint on the body z of a VertexPose. */
class EdgeBaroZ : public g2o::BaseUnaryEdge<1, double, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeBaroZ() {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError()
    {
        const VertexPose* vertexPose = static_cast<const VertexPose*>(_vertices[0]);
        _error << vertexPose->estimate().twb[2] - _measurement;
    }

    virtual void linearizeOplus()
    {
        const VertexPose* vertexPose = static_cast<const VertexPose*>(_vertices[0]);
        _jacobianOplusXi.setZero();
        // body-frame translation update: d twb.z / d ut = Rwb.row(2)
        _jacobianOplusXi.block<1, 3>(0, 3) = vertexPose->estimate().Rwb.row(2);
    }
};

/** Barometric altitude-difference constraint on (gravity direction, scale) during IMU
 *  initialization:  r = s * up_w^T (twb_i - twb_ref) - (baro_i - baro_ref),  up_w = Rwg * e_z.
 *  Makes the monocular scale observable from the barometer even without accelerometer excitation
 *  (high-altitude cruise, where the accel-only init collapses with "scale too small").
 *  Jacobians: g2o numeric differentiation (init-only cost, negligible). */
class EdgeBaroScaleGDir : public g2o::BaseBinaryEdge<1, double, VertexGDir, VertexScale>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeBaroScaleGDir(const Eigen::Vector3d& deltaPosition) : mDeltaPosition(deltaPosition) {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError()
    {
        const VertexGDir* vertexGDir = static_cast<const VertexGDir*>(_vertices[0]);
        const VertexScale* vertexScale = static_cast<const VertexScale*>(_vertices[1]);
        const Eigen::Vector3d upWorld = vertexGDir->estimate().Rwg.col(2);
        _error << vertexScale->estimate() * upWorld.dot(mDeltaPosition) - _measurement;
    }

private:
    const Eigen::Vector3d mDeltaPosition;
};

}  // namespace ORB_SLAM3

#endif  // BAROFUSION_H
