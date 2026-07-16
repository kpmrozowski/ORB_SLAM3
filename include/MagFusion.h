/**
 * MagFusion — env-gated magnetometer yaw fusion for the inertial bundle adjustments.
 *
 * ORB_MAG_CSV=<path>        ref_mag.csv ("#timestamp [ns],mx,my,mz", rebased ns matching image
 *                           stamps; the vector is the Earth field in the SLAM BODY (IMU) frame,
 *                           already vehicle->IMU aligned and unit-normalized by export_mag_csv.py)
 * ORB_MAG_SIGMA_DEG=<deg>   1-sigma of one yaw constraint (default 5.0 deg)
 *
 * After IMU init the world frame is gravity-aligned (+z up) and, if this fusion is enabled,
 * additionally yaw-aligned so magnetic north lies along world +x (see LocalMapping::InitializeIMU).
 * The Earth field is CONSTANT in the world frame, so h = R_wb * m_body has a fixed azimuth: any
 * per-keyframe deviation of atan2(h.y, h.x) from the window reference is genuine VIO yaw drift.
 * Like the barometric datum, the yaw reference is recomputed every optimization (newest fixed
 * keyframe, else window median), so the edges suppress RELATIVE yaw drift and stay gauge-safe
 * under later ApplyScaledRotation / ScaleRefinement global similarity updates.
 */

#ifndef MAGFUSION_H
#define MAGFUSION_H

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "G2oTypes.h"

namespace ORB_SLAM3
{

constexpr double kMagPi = 3.14159265358979323846;

/** Wrap an angle to (-pi, pi]. */
inline double NormalizeAngle(double angle)
{
    while (angle > kMagPi)
    {
        angle -= 2.0 * kMagPi;
    }
    while (angle <= -kMagPi)
    {
        angle += 2.0 * kMagPi;
    }
    return angle;
}

class MagFusion
{
public:
    static MagFusion& Instance()
    {
        static MagFusion instance;
        return instance;
    }

    bool Enabled() const { return !mTimesSec.empty(); }

    double SigmaRad() const { return mSigmaDeg * kMagPi / 180.0; }

    double SigmaDeg() const { return mSigmaDeg; }

    /** Linear interpolation of the body-frame unit field at t [s]; false outside the record. */
    bool FieldAt(const double timeSec, Eigen::Vector3d& field) const
    {
        if (mTimesSec.empty() || timeSec < mTimesSec.front() - 0.5 || timeSec > mTimesSec.back() + 0.5)
        {
            return false;
        }
        const std::vector<double>::const_iterator upper =
            std::lower_bound(mTimesSec.begin(), mTimesSec.end(), timeSec);
        if (upper == mTimesSec.begin())
        {
            field = mFields.front();
        }
        else if (upper == mTimesSec.end())
        {
            field = mFields.back();
        }
        else
        {
            const size_t hi = upper - mTimesSec.begin();
            const size_t lo = hi - 1;
            const double span = mTimesSec[hi] - mTimesSec[lo];
            const double weight = span > 0.0 ? (timeSec - mTimesSec[lo]) / span : 0.0;
            field = (1.0 - weight) * mFields[lo] + weight * mFields[hi];
        }
        const double norm = field.norm();
        if (norm < 1e-9)
        {
            return false;
        }
        field /= norm;
        return true;
    }

private:
    MagFusion() : mSigmaDeg(5.0)
    {
        const char* csvPath = getenv("ORB_MAG_CSV");
        if (!csvPath)
        {
            return;
        }
        const char* sigmaEnv = getenv("ORB_MAG_SIGMA_DEG");
        if (sigmaEnv)
        {
            mSigmaDeg = atof(sigmaEnv);
        }
        std::ifstream file(csvPath);
        std::string line;
        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }
            std::stringstream stream(line);
            std::string timeField;
            std::string valueX;
            std::string valueY;
            std::string valueZ;
            if (!std::getline(stream, timeField, ',') || !std::getline(stream, valueX, ',') ||
                !std::getline(stream, valueY, ',') || !std::getline(stream, valueZ, ','))
            {
                continue;
            }
            mTimesSec.push_back(std::stod(timeField) * 1e-9);
            mFields.emplace_back(std::stod(valueX), std::stod(valueY), std::stod(valueZ));
        }
        std::cout << "MagFusion: " << mTimesSec.size() << " mag samples from " << csvPath
                  << " sigma " << mSigmaDeg << " deg" << std::endl;
    }

    std::vector<double> mTimesSec;
    std::vector<Eigen::Vector3d> mFields;
    double mSigmaDeg;
};

/** Unary magnetometer-yaw constraint on the orientation of a VertexPose. The Earth field in the
 *  world frame is h = Rwb * mFieldBody; its azimuth atan2(h.y, h.x) is compared to the window yaw
 *  reference. Jacobians: g2o numeric differentiation (1-D unary edge, negligible cost — verified
 *  not to affect the intermittent oversubscription crash: an analytic form gave the same rate). */
class EdgeMagYaw : public g2o::BaseUnaryEdge<1, double, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeMagYaw(const Eigen::Vector3d& fieldBody) : mFieldBody(fieldBody) {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError()
    {
        const VertexPose* vertexPose = static_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector3d fieldWorld = vertexPose->estimate().Rwb * mFieldBody;
        const double yaw = std::atan2(fieldWorld.y(), fieldWorld.x());
        _error << NormalizeAngle(yaw - _measurement);
    }

private:
    const Eigen::Vector3d mFieldBody;
};

}  // namespace ORB_SLAM3

#endif  // MAGFUSION_H
