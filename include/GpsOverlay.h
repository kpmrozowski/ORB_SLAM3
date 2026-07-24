/**
 * GpsOverlay — env-gated GPS(ENU)-vs-SLAM overlay for the Pangolin map viewer.
 *
 * ORB_GPS_CSV=<path>    GPS[0] (spoofed instance) CSV "t_rebased_ns,e,n,u,valid".
 * ORB_GPS1_CSV=<path>   GPS[1] (NORA, trusted receiver) CSV, same schema, same ENU frame.
 *
 * With BOTH env vars unset the overlay is completely inactive (Enabled()==false) and the viewer is
 * byte-for-byte stock behaviour. Either env absent leaves that one track fully inactive. Parsing
 * never throws: a missing / short / garbage CSV simply yields an empty sample set (that track
 * inactive). This is a display-only feature: the sole consumer is MapDrawer::DrawGPS — GPS is never
 * fed back into the estimator.
 *
 * Alignment fits a single Sim3 T_slam_gps (Eigen::umeyama, with scaling) from GPS-ENU (linearly
 * interpolated at keyframe timestamps) to SLAM keyframe camera centres, recomputed at most every
 * ~2 s of wall-clock. The fit SOURCE is GPS[0] when present — GPS[1] then shares that exact
 * transform, so the two tracks deliberately diverge wherever GPS[0] is spoofed (both CSVs are
 * emitted in one shared ENU frame). When GPS[0] is absent, the fit is computed from GPS[1]↔keyframe
 * correspondences instead (same code path, different source), so GPS[0]-less flights still align.
 * The refit runs on the Viewer thread from MapDrawer::DrawGPS and caches the transform between
 * refits, so it stays cheap.
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
        // GPS[0] trajectory as one polyline per contiguous run of valid samples: break the
        // GL_LINE_STRIP between runs so invalid gaps (teleports) are never connected.
        std::vector<std::vector<Eigen::Vector3f>> trajSegments;
        // GPS[1] (NORA) trajectory, split wherever consecutive samples are > 5 s apart (GPS[1]
        // carries no spoof flag — a wide sample gap is a dropout, never a straight-line teleport).
        std::vector<std::vector<Eigen::Vector3f>> traj1Segments;
        // GPS[0] correspondence pairs (kf_centre_slam, gps_in_slam) for the keyframes used in the fit.
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> correspondences;
        // GPS[1] correspondence pairs (kf_centre_slam, gps1_in_slam) for keyframes whose nearest
        // GPS[1] sample is within 5 s (same gap discipline as traj1Segments — no line over a hole).
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> correspondences1;
        // True once a valid Sim3 alignment has been computed at least once.
        bool valid = false;
    };

    static GpsOverlay& Instance()
    {
        static GpsOverlay instance;
        return instance;
    }

    /** False (inactive) when neither ORB_GPS_CSV nor ORB_GPS1_CSV yielded any usable samples. */
    bool Enabled() const { return !mSamples.empty() || !mSamples1.empty(); }

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

    /** Parse a GPS ENU CSV from `csvPath`; empty (inactive) on null path / unopenable / garbage.
     *  Logs the loaded sample count under `label` when the path is non-null (fail-open otherwise). */
    static std::vector<Sample> LoadSamples(const char* csvPath, const char* label);

    /** Linear interpolation of `samples` ENU at time [s]; false outside the record or when either
     *  bracketing sample is flagged invalid. */
    static bool InterpolateEnu(const std::vector<Sample>& samples, const double timeSec,
                               Eigen::Vector3d& enu);

    /** Seconds from `timeSec` to the nearest sample in `samples` (infinity when empty). */
    static double NearestSampleGap(const std::vector<Sample>& samples, const double timeSec);

    /** Recompute mT_slam_gps from keyframe correspondences — fit against GPS[0] when present, else
     *  GPS[1] (needs >= 3 correspondences, else keep the last good transform / stay inactive). */
    void Refit(const std::vector<KeyFramePose>& keyframePoses);

    /** Build the drawable geometry (both tracks) from the cached transform. */
    DrawData BuildDrawData(const std::vector<KeyFramePose>& keyframePoses) const;

    std::vector<Sample> mSamples;   // GPS[0] (spoofed instance)
    std::vector<Sample> mSamples1;  // GPS[1] (NORA, trusted receiver)
    Eigen::Matrix4d mT_slam_gps;
    bool mHasTransform;
    bool mFirstRefit;
    std::chrono::steady_clock::time_point mLastRefit;
    std::mutex mMutex;
};

}  // namespace ORB_SLAM3

#endif  // GPSOVERLAY_H
