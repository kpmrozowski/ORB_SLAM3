/**
 * GpsOverlay — see include/GpsOverlay.h.
 *
 * Fail-open throughout: nothing here throws or crashes on a missing / short / garbage CSV; the
 * overlay simply stays inactive (Enabled()==false) so the viewer keeps its stock behaviour.
 */

#include "GpsOverlay.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// Recompute the Sim3 alignment at most this often (wall-clock throttle).
const int kRefitIntervalMs = 2000;

// Apply a Sim3 (scale*R | t) 4x4 to an ENU point, returning the result in the SLAM/GL frame.
Eigen::Vector3f ApplyTransform(const Eigen::Matrix4d& transform, const Eigen::Vector3d& enu)
{
    const Eigen::Vector4d homogeneous(enu.x(), enu.y(), enu.z(), 1.0);
    const Eigen::Vector4d transformed = transform * homogeneous;
    return transformed.head<3>().cast<float>();
}

}  // namespace

namespace ORB_SLAM3
{

GpsOverlay::GpsOverlay()
    : mT_slam_gps(Eigen::Matrix4d::Identity()), mHasTransform(false), mFirstRefit(true)
{
    const char* csvPath = getenv("ORB_GPS_CSV");
    if (!csvPath)
    {
        return;
    }
    std::ifstream file(csvPath);
    if (!file.is_open())
    {
        return;
    }
    std::string line;
    while (std::getline(file, line))
    {
        // Skip blank lines and comment / header lines (a header starts with '#' or a non-numeric
        // first field, which trips the std::stod below and gets skipped there).
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::stringstream stream(line);
        std::string field;
        std::vector<std::string> fields;
        while (std::getline(stream, field, ','))
        {
            fields.push_back(field);
        }
        if (fields.size() < 5)
        {
            continue;  // short / malformed line
        }
        Sample sample;
        try
        {
            sample.timeSec = std::stod(fields[0]) * 1e-9;
            sample.enu = Eigen::Vector3d(std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3]));
            sample.valid = std::stod(fields[4]) != 0.0;
        }
        catch (const std::exception&)
        {
            continue;  // header row or garbage numeric — skip, stay fail-open
        }
        mSamples.push_back(sample);
    }
    std::cout << "GpsOverlay: " << mSamples.size() << " GPS samples from " << csvPath << std::endl;
}

bool GpsOverlay::InterpolateEnu(const double timeSec, Eigen::Vector3d& enu) const
{
    if (mSamples.empty() || timeSec < mSamples.front().timeSec || timeSec > mSamples.back().timeSec)
    {
        return false;
    }
    const std::vector<Sample>::const_iterator upper =
        std::lower_bound(mSamples.begin(), mSamples.end(), timeSec,
                         [](const Sample& sample, const double value) { return sample.timeSec < value; });
    if (upper == mSamples.begin())
    {
        if (!mSamples.front().valid)
        {
            return false;
        }
        enu = mSamples.front().enu;
        return true;
    }
    const std::vector<Sample>::const_iterator lower = upper - 1;
    if (!lower->valid || !upper->valid)
    {
        return false;
    }
    const double span = upper->timeSec - lower->timeSec;
    const double weight = span > 0.0 ? (timeSec - lower->timeSec) / span : 0.0;
    enu = (1.0 - weight) * lower->enu + weight * upper->enu;
    return true;
}

void GpsOverlay::Refit(const std::vector<KeyFramePose>& keyframePoses)
{
    std::vector<Eigen::Vector3d> gpsPoints;
    std::vector<Eigen::Vector3d> slamPoints;
    gpsPoints.reserve(keyframePoses.size());
    slamPoints.reserve(keyframePoses.size());
    for (const KeyFramePose& keyframePose : keyframePoses)
    {
        Eigen::Vector3d enu;
        if (InterpolateEnu(keyframePose.first, enu))
        {
            gpsPoints.push_back(enu);
            slamPoints.push_back(keyframePose.second.cast<double>());
        }
    }
    if (gpsPoints.size() < 3)
    {
        return;  // too few correspondences — keep the last good transform (or stay inactive)
    }
    Eigen::Matrix<double, 3, Eigen::Dynamic> source(3, gpsPoints.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> destination(3, slamPoints.size());
    for (size_t index = 0; index < gpsPoints.size(); ++index)
    {
        source.col(index) = gpsPoints[index];
        destination.col(index) = slamPoints[index];
    }
    // src = GPS ENU, dst = SLAM keyframe camera centres → Sim3 mapping GPS into the SLAM frame.
    mT_slam_gps = Eigen::umeyama(source, destination, true);
    mHasTransform = true;
}

GpsOverlay::DrawData GpsOverlay::BuildDrawData(const std::vector<KeyFramePose>& keyframePoses) const
{
    DrawData drawData;
    drawData.valid = mHasTransform;
    if (!mHasTransform)
    {
        return drawData;
    }
    // (a) trajectory polyline: one segment per contiguous run of valid samples (gaps at valid==0).
    std::vector<Eigen::Vector3f> segment;
    for (const Sample& sample : mSamples)
    {
        if (sample.valid)
        {
            segment.push_back(ApplyTransform(mT_slam_gps, sample.enu));
        }
        else if (!segment.empty())
        {
            drawData.trajSegments.push_back(segment);
            segment.clear();
        }
    }
    if (!segment.empty())
    {
        drawData.trajSegments.push_back(segment);
    }
    // (b) correspondence pairs (kf_centre_slam, gps_in_slam) for the used keyframes.
    for (const KeyFramePose& keyframePose : keyframePoses)
    {
        Eigen::Vector3d enu;
        if (InterpolateEnu(keyframePose.first, enu))
        {
            drawData.correspondences.push_back(
                std::make_pair(keyframePose.second, ApplyTransform(mT_slam_gps, enu)));
        }
    }
    return drawData;
}

GpsOverlay::DrawData GpsOverlay::ComputeDrawData(const std::vector<KeyFramePose>& keyframePoses)
{
    std::lock_guard<std::mutex> lock(mMutex);
    if (mSamples.empty())
    {
        return DrawData();
    }
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (mFirstRefit || (now - mLastRefit) >= std::chrono::milliseconds(kRefitIntervalMs))
    {
        Refit(keyframePoses);
        mLastRefit = now;
        mFirstRefit = false;
    }
    return BuildDrawData(keyframePoses);
}

}  // namespace ORB_SLAM3
