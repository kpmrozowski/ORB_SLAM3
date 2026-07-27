/**
 * Task P3d unit test: PayloadSpill record roundtrip.
 *
 * Asserts that a KeyFrame payload (descriptors, undistorted keypoints, DBoW2 FeatureVector) written
 * through the fixed-layout binary record and read back is BYTE-EXACT, and that crc32 detects
 * corruption. assert-main style (no gtest): returns 0 on success, aborts with a message otherwise.
 * This is the unit seam the plan requires before the engine integration (superpowers TDD).
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "PayloadSpill.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

using namespace ORB_SLAM3;

namespace
{

int g_checks = 0;

void Check(const bool condition, const char* const message)
{
    ++g_checks;
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::abort();
    }
}

// Build a deterministic but non-trivial descriptor matrix (n x 32, CV_8U).
cv::Mat MakeDescriptors(const int feature_count)
{
    cv::Mat descriptors(feature_count, 32, CV_8U);
    for (int row = 0; row < feature_count; ++row)
    {
        for (int col = 0; col < 32; ++col)
        {
            descriptors.at<std::uint8_t>(row, col) =
                static_cast<std::uint8_t>((row * 31 + col * 7 + 13) & 0xFF);
        }
    }
    return descriptors;
}

std::vector<cv::KeyPoint> MakeKeys(const int feature_count)
{
    std::vector<cv::KeyPoint> keys(feature_count);
    for (int index = 0; index < feature_count; ++index)
    {
        cv::KeyPoint& keypoint = keys[index];
        keypoint.pt.x = 1.5f * index + 0.25f;
        keypoint.pt.y = -3.0f * index + 7.75f;
        keypoint.size = 31.0f + (index % 5);
        keypoint.angle = (index % 360) - 1.0f;
        keypoint.response = 0.001f * index;
        keypoint.octave = index % 8;
        keypoint.class_id = -1 + (index % 3);
    }
    return keys;
}

DBoW2::FeatureVector MakeFeatVec(const int feature_count)
{
    DBoW2::FeatureVector feature_vector;
    // A handful of nodes with varying-length feature-index lists (ascending node ids).
    for (int node = 0; node < 40; ++node)
    {
        const DBoW2::NodeId node_id = 100u + node * 3u;
        const int list_length = 1 + (node % 6);
        for (int entry = 0; entry < list_length; ++entry)
        {
            feature_vector.addFeature(node_id, static_cast<unsigned int>((node + entry) % feature_count));
        }
    }
    return feature_vector;
}

bool KeysEqual(const cv::KeyPoint& first, const cv::KeyPoint& second)
{
    return first.pt.x == second.pt.x && first.pt.y == second.pt.y && first.size == second.size &&
           first.angle == second.angle && first.response == second.response &&
           first.octave == second.octave && first.class_id == second.class_id;
}

}  // namespace

int main()
{
    const int feature_count = 1000;
    const cv::Mat descriptors = MakeDescriptors(feature_count);
    const std::vector<cv::KeyPoint> keys = MakeKeys(feature_count);
    const DBoW2::FeatureVector feature_vector = MakeFeatVec(feature_count);

    // 1. Body (de)serialization is byte-exact (no file involved).
    std::vector<std::uint8_t> body;
    spill::SerializePayloadBody(descriptors, keys, feature_vector, body);
    Check(!body.empty(), "serialized body is non-empty");

    cv::Mat desc_out;
    std::vector<cv::KeyPoint> keys_out;
    DBoW2::FeatureVector featvec_out;
    Check(spill::DeserializePayloadBody(body.data(), body.size(), feature_count, desc_out, keys_out,
                                        featvec_out),
          "deserialize body succeeds");

    Check(desc_out.rows == descriptors.rows && desc_out.cols == descriptors.cols,
          "descriptor dims match");
    Check(cv::countNonZero(desc_out != descriptors) == 0, "descriptor bytes byte-exact");

    Check(keys_out.size() == keys.size(), "keypoint count matches");
    for (std::size_t index = 0; index < keys.size(); ++index)
    {
        Check(KeysEqual(keys[index], keys_out[index]), "keypoint byte-exact");
    }

    Check(featvec_out.size() == feature_vector.size(), "featvec node count matches");
    for (DBoW2::FeatureVector::const_iterator node_it = feature_vector.begin();
         node_it != feature_vector.end(); ++node_it)
    {
        const DBoW2::FeatureVector::const_iterator found = featvec_out.find(node_it->first);
        Check(found != featvec_out.end(), "featvec node present");
        Check(found->second == node_it->second, "featvec index list byte-exact");
    }

    // 2. Full record roundtrip through the PayloadSpill file (header + crc framing).
    const std::string path = "/tmp/orbmem_spill_roundtrip_test.bin";
    {
        PayloadSpill spill(path);
        Check(spill.Ok(), "spill file opened");

        const std::uint64_t kf_id = 4242;
        Check(spill.WriteRecord(kf_id, feature_count, body), "record written");
        Check(spill.HasRecord(kf_id), "record indexed");
        Check(!spill.HasRecord(9999), "absent record reports missing");

        // Write-once: a second write for the same id must not append a new record.
        const std::uint64_t before = spill.BytesWritten();
        Check(spill.WriteRecord(kf_id, feature_count, body), "second write is a no-op success");
        Check(spill.BytesWritten() == before, "write-once: no duplicate record appended");

        std::vector<std::uint8_t> body_read;
        Check(spill.ReadRecord(kf_id, feature_count, body_read), "record read back");
        Check(body_read == body, "record body byte-exact through the file");

        cv::Mat desc2;
        std::vector<cv::KeyPoint> keys2;
        DBoW2::FeatureVector featvec2;
        Check(spill::DeserializePayloadBody(body_read.data(), body_read.size(), feature_count, desc2,
                                            keys2, featvec2),
              "deserialize file-read body");
        Check(cv::countNonZero(desc2 != descriptors) == 0, "descriptors byte-exact after file rt");
        Check(keys2.size() == keys.size(), "keys count after file rt");
    }

    // 3. crc32 catches corruption (a flipped byte changes the checksum).
    {
        const std::uint32_t crc_a = spill::Crc32(body.data(), body.size());
        std::vector<std::uint8_t> corrupted = body;
        corrupted[corrupted.size() / 2] ^= 0x01;
        const std::uint32_t crc_b = spill::Crc32(corrupted.data(), corrupted.size());
        Check(crc_a != crc_b, "crc32 differs on a single-bit corruption");
    }

    std::remove(path.c_str());
    std::printf("payload_spill_roundtrip: OK (%d checks passed)\n", g_checks);
    return 0;
}
