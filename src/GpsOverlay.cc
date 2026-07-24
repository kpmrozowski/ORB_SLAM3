/**
 * GpsOverlay — see include/GpsOverlay.h.
 *
 * Fail-open throughout: nothing here throws or crashes on a missing / short / garbage CSV; the
 * overlay simply stays inactive (Enabled()==false) so the viewer keeps its stock behaviour.
 */

#include "GpsOverlay.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Recompute the Sim3 alignment at most this often (wall-clock throttle).
const int kRefitIntervalMs = 2000;

// GPS[1] carries no spoof/valid flag (it is the trusted receiver), so its polyline is split — and
// its correspondence lines are dropped — wherever the sample cadence exceeds this many seconds: a
// wide gap is a dropout, never a straight-line teleport.
const double kGps1GapSplitSec = 5.0;

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

std::vector<GpsOverlay::Sample> GpsOverlay::LoadSamples(const char* csvPath, const char* label)
{
    std::vector<Sample> samples;
    if (!csvPath)
    {
        return samples;
    }
    std::ifstream file(csvPath);
    if (!file.is_open())
    {
        return samples;
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
        samples.push_back(sample);
    }
    std::cout << "GpsOverlay: " << samples.size() << " " << label << " samples from " << csvPath << std::endl;
    return samples;
}

GpsOverlay::GpsOverlay()
    : mT_slam_gps(Eigen::Matrix4d::Identity()), mHasTransform(false), mFirstRefit(true)
{
    mSamples = LoadSamples(getenv("ORB_GPS_CSV"), "GPS[0]");
    mSamples1 = LoadSamples(getenv("ORB_GPS1_CSV"), "GPS[1]");
}

bool GpsOverlay::InterpolateEnu(const std::vector<Sample>& samples, const double timeSec,
                                Eigen::Vector3d& enu)
{
    if (samples.empty() || timeSec < samples.front().timeSec || timeSec > samples.back().timeSec)
    {
        return false;
    }
    const std::vector<Sample>::const_iterator upper =
        std::lower_bound(samples.begin(), samples.end(), timeSec,
                         [](const Sample& sample, const double value) { return sample.timeSec < value; });
    if (upper == samples.begin())
    {
        if (!samples.front().valid)
        {
            return false;
        }
        enu = samples.front().enu;
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

double GpsOverlay::NearestSampleGap(const std::vector<Sample>& samples, const double timeSec)
{
    if (samples.empty())
    {
        return std::numeric_limits<double>::infinity();
    }
    const std::vector<Sample>::const_iterator upper =
        std::lower_bound(samples.begin(), samples.end(), timeSec,
                         [](const Sample& sample, const double value) { return sample.timeSec < value; });
    double gap = std::numeric_limits<double>::infinity();
    if (upper != samples.end())
    {
        gap = std::min(gap, std::abs(upper->timeSec - timeSec));
    }
    if (upper != samples.begin())
    {
        gap = std::min(gap, std::abs(timeSec - (upper - 1)->timeSec));
    }
    return gap;
}

void GpsOverlay::Refit(const std::vector<KeyFramePose>& keyframePoses)
{
    // Fit source: GPS[0] when present (GPS[1] then SHARES this transform, so any spoof-induced
    // GPS[0]-vs-GPS[1] divergence stays visible); otherwise fit on GPS[1] so GPS[0]-less flights
    // still get a truth overlay. Same code path, different source.
    const std::vector<Sample>& fitSamples = !mSamples.empty() ? mSamples : mSamples1;
    std::vector<Eigen::Vector3d> gpsPoints;
    std::vector<Eigen::Vector3d> slamPoints;
    gpsPoints.reserve(keyframePoses.size());
    slamPoints.reserve(keyframePoses.size());
    for (const KeyFramePose& keyframePose : keyframePoses)
    {
        Eigen::Vector3d enu;
        if (InterpolateEnu(fitSamples, keyframePose.first, enu))
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
    // (a) GPS[0] trajectory polyline: one segment per contiguous run of valid samples (gaps at
    //     valid==0). Inactive (empty) when ORB_GPS_CSV was unset.
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
    // (b) GPS[1] trajectory polyline: split wherever the sample cadence exceeds the gap threshold
    //     (GPS[1] carries no spoof flag; a stray invalid sample also breaks the run defensively).
    std::vector<Eigen::Vector3f> segment1;
    double previousTime1 = 0.0;
    for (const Sample& sample : mSamples1)
    {
        const bool gap = !segment1.empty() && (sample.timeSec - previousTime1) > kGps1GapSplitSec;
        if (gap || !sample.valid)
        {
            if (!segment1.empty())
            {
                drawData.traj1Segments.push_back(segment1);
                segment1.clear();
            }
            if (!sample.valid)
            {
                continue;
            }
        }
        segment1.push_back(ApplyTransform(mT_slam_gps, sample.enu));
        previousTime1 = sample.timeSec;
    }
    if (!segment1.empty())
    {
        drawData.traj1Segments.push_back(segment1);
    }
    // (c) GPS[0] correspondence pairs (kf_centre_slam, gps_in_slam) for the used keyframes.
    for (const KeyFramePose& keyframePose : keyframePoses)
    {
        Eigen::Vector3d enu;
        if (InterpolateEnu(mSamples, keyframePose.first, enu))
        {
            drawData.correspondences.push_back(
                std::make_pair(keyframePose.second, ApplyTransform(mT_slam_gps, enu)));
        }
    }
    // (d) GPS[1] correspondence pairs, only where a GPS[1] sample sits within the gap threshold of
    //     the keyframe stamp (same >5 s discipline as the strip — never a line across a dropout).
    for (const KeyFramePose& keyframePose : keyframePoses)
    {
        Eigen::Vector3d enu;
        if (InterpolateEnu(mSamples1, keyframePose.first, enu)
            && NearestSampleGap(mSamples1, keyframePose.first) <= kGps1GapSplitSec)
        {
            drawData.correspondences1.push_back(
                std::make_pair(keyframePose.second, ApplyTransform(mT_slam_gps, enu)));
        }
    }
    return drawData;
}

GpsOverlay::DrawData GpsOverlay::ComputeDrawData(const std::vector<KeyFramePose>& keyframePoses)
{
    std::lock_guard<std::mutex> lock(mMutex);
    if (mSamples.empty() && mSamples1.empty())
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
