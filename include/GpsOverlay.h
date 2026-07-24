/**
 * GpsOverlay — env-gated GPS(ENU)-vs-SLAM overlay for the Pangolin map viewer.
 *
 * ORB_GPS_CSV=<path>   CSV "t_rebased_ns,e,n,u,valid" (rebased ns matching image stamps).
 *
 * With ORB_GPS_CSV unset the overlay is completely inactive (Enabled()==false) and the viewer
 * is byte-for-byte stock behaviour. Parsing never throws: a missing / short / garbage CSV simply
 * yields an empty sample set (inactive). Alignment fits a Sim3 T_slam_gps (Eigen::umeyama, with
 * scaling) from GPS-ENU (linearly interpolated at keyframe timestamps) to SLAM keyframe camera
 * centres, recomputed at most every ~2 s of wall-clock. The refit runs on the Viewer thread from
 * MapDrawer::DrawGPS, so it caches the transform between refits and stays cheap.
 *
 * Kept self-contained (no KeyFrame.h) to avoid include cycles: the caller pulls keyframe
 * (timestamp, camera-centre) pairs and hands them in; GpsOverlay owns interpolation + umeyama +
 * caching and returns geometry already expressed in the SLAM/map GL frame.
 */

#ifndef GPSOVERLAY_H
#define GPSOVERLAY_H

#include <chrono>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM3
{

class GpsOverlay
{
public:
    /** Prepared per-keyframe correspondence input: (timestamp [s], camera centre in SLAM/world). */
    typedef std::pair<double, Eigen::Vector3f> KeyFramePose;

    /** Everything DrawGPS needs, already expressed in the SLAM/map GL frame. */
    struct DrawData
    {
        // GPS trajectory as one polyline per contiguous run of valid samples: break the
        // GL_LINE_STRIP between runs so invalid gaps (teleports) are never connected.
        std::vector<std::vector<Eigen::Vector3f>> trajSegments;
        // Correspondence pairs (kf_centre_slam, gps_in_slam) for the keyframes used in the fit.
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> correspondences;
        // True once a valid Sim3 alignment has been computed at least once.
        bool valid = false;
    };

    static GpsOverlay& Instance()
    {
        static GpsOverlay instance;
        return instance;
    }

    /** False (inactive) when ORB_GPS_CSV was unset or the CSV yielded no usable samples. */
    bool Enabled() const { return !mSamples.empty(); }

    /**
     * Refit (throttled to ~2 s of wall-clock) and return the drawable overlay in the SLAM frame.
     * Cheap between refits (reuses the cached transform). Meant to be called from the Viewer thread.
     */
    DrawData ComputeDrawData(const std::vector<KeyFramePose>& keyframePoses);

private:
    struct Sample
    {
        double timeSec;
        Eigen::Vector3d enu;
        bool valid;
    };

    GpsOverlay();
    GpsOverlay(const GpsOverlay&) = delete;
    GpsOverlay& operator=(const GpsOverlay&) = delete;

    /** Linear interpolation of the GPS ENU at time [s]; false outside the record or when either
     *  bracketing sample is flagged invalid. */
    bool InterpolateEnu(const double timeSec, Eigen::Vector3d& enu) const;

    /** Recompute mT_slam_gps from keyframe correspondences (needs >= 3, else keep the last good). */
    void Refit(const std::vector<KeyFramePose>& keyframePoses);

    /** Build the drawable geometry from the cached transform. */
    DrawData BuildDrawData(const std::vector<KeyFramePose>& keyframePoses) const;

    std::vector<Sample> mSamples;
    Eigen::Matrix4d mT_slam_gps;
    bool mHasTransform;
    bool mFirstRefit;
    std::chrono::steady_clock::time_point mLastRefit;
    std::mutex mMutex;
};

}  // namespace ORB_SLAM3

#endif  // GPSOVERLAY_H
