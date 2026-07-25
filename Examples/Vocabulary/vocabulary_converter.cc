/**
 * vocabulary_converter — Task P2 standalone tool.
 *
 * Converts a DBoW2 text vocabulary (ORBvoc.txt) into the compact mmap .cvoc sidecar (see
 * include/CompactVocabulary.h for the on-disk layout), and, with --verify, PROVES the
 * conversion is bit-exact for BoW transform()/score() before System.cc's ORB_VOC_COMPACT path
 * is ever allowed to depend on it. This is the primary safety net for Task P2: a single bit of
 * divergence here would silently corrupt place recognition (SearchByBoW/relocalization/loop
 * closure), so --verify asserts exact equality rather than approximate closeness anywhere.
 *
 * Usage: vocabulary_converter <text_vocab.txt> <output.cvoc> [--verify] [--frame-image <path>]
 *
 * --verify runs three checks against the freshly-written .cvoc:
 *   1. size() parity.
 *   2. 10,000 independently-seeded random 32-byte descriptors, each transformed alone (a batch
 *      of exactly one feature) through both vocabularies - this isolates every single
 *      descriptor's own (word_id, node_id, weight) triple for maximum diagnostic precision.
 *   3. One real flight image (default: a frame from the 212_golem27 dataset used by the
 *      project's own determinism gate), run through the actual ORBextractor with the same
 *      config the pipeline uses, transformed as ONE aggregate batch (mirroring exactly how
 *      Frame::ComputeBoW/KeyFrame::ComputeBoW call the vocabulary in production), and scored
 *      against a second synthetic "frame" - this exercises the addWeight() accumulation/
 *      normalization path and the score() delegation that check 2's singleton batches cannot.
 *      Soft-skipped (not a hard failure) if the image is unavailable, since it is a bonus check
 *      on top of the mandatory random-descriptor sweep.
 */

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "CompactVocabulary.h"
#include "ORBextractor.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace
{

// Deliberately spelled out here rather than taken from ORB_SLAM3::DBoW2Vocabulary
// (include/ORBVocabulary.h): this tool only needs the stock DBoW2 template's
// loadFromTextFile/transform/score/size, so it stays buildable standalone against just
// Thirdparty/DBoW2 - independent of ORBVocabulary.h's dispatch-class wrapper.
using StockVocabulary = DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;

// Matches Frame::ComputeBoW/KeyFrame::ComputeBoW (both hardcode levelsup=4) and
// eval_fast/config_212_golem27.yaml's ORBextractor.* block, so the "real frame" check
// exercises exactly the production configuration.
constexpr int kLevelsUp = 4;
constexpr unsigned int kRandomSampleCount = 10000U;
const char* const kDefaultFrameImage =
    "/home/kmro/praca/dev/orbslam3-eval/dataset/hist_212_golem27/mav0/cam0/data/0.png";

cv::Mat RandomDescriptor(std::mt19937_64& generator)
{
    std::uniform_int_distribution<int> byte_distribution(0, 255);
    cv::Mat descriptor(1, 32, CV_8U);
    unsigned char* const data = descriptor.ptr<unsigned char>(0);
    std::generate(data, data + 32, [&generator, &byte_distribution]() {
        return static_cast<unsigned char>(byte_distribution(generator));
    });
    return descriptor;
}

// Bit-level equality (not merely numeric ==) on every shared word's weight, as the brief
// requires: two different bit patterns that happen to compare == (e.g. +0.0 and -0.0) would
// still fail this check, which is intentionally stricter than std::map::operator==.
bool BowVectorsBitExact(const DBoW2::BowVector& lhs, const DBoW2::BowVector& rhs)
{
    if (lhs.size() != rhs.size())
    {
        return false;
    }
    return std::all_of(lhs.begin(), lhs.end(), [&rhs](const DBoW2::BowVector::value_type& entry) {
        const DBoW2::BowVector::const_iterator found = rhs.find(entry.first);
        return found != rhs.end() && std::memcmp(&entry.second, &found->second, sizeof(double)) == 0;
    });
}

// NodeId keys and per-node feature-index vectors are all integers, so plain == is exact here -
// no bit-level subtlety like the double weights above.
bool FeatureVectorsExact(const DBoW2::FeatureVector& lhs, const DBoW2::FeatureVector& rhs)
{
    return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

// Runs `count` independent single-descriptor transforms through both vocabularies. Returns the
// number of bit-exact matches; logs full detail on the first mismatch only (to avoid flooding
// stderr if something is systematically wrong).
unsigned int VerifyRandomDescriptors(const StockVocabulary& text_vocabulary,
                                      const ORB_SLAM3::CompactVocabulary& compact_vocabulary,
                                      const unsigned int count)
{
    std::mt19937_64 generator(0xC0FFEEULL);
    unsigned int match_count = 0U;
    bool first_mismatch_reported = false;
    for (unsigned int sample_index = 0U; sample_index < count; ++sample_index)
    {
        const std::vector<cv::Mat> single_feature{RandomDescriptor(generator)};

        DBoW2::BowVector bow_text;
        DBoW2::FeatureVector feature_vector_text;
        text_vocabulary.transform(single_feature, bow_text, feature_vector_text, kLevelsUp);

        DBoW2::BowVector bow_compact;
        DBoW2::FeatureVector feature_vector_compact;
        compact_vocabulary.Transform(single_feature, bow_compact, feature_vector_compact, kLevelsUp);

        const bool bow_ok = BowVectorsBitExact(bow_text, bow_compact);
        const bool feature_ok = FeatureVectorsExact(feature_vector_text, feature_vector_compact);
        if (bow_ok && feature_ok)
        {
            ++match_count;
        }
        else if (!first_mismatch_reported)
        {
            first_mismatch_reported = true;
            std::cerr << "vocabulary_converter --verify: MISMATCH at random sample " << sample_index
                      << " (bow_ok=" << bow_ok << " feature_ok=" << feature_ok << ")" << std::endl;
        }
    }
    return match_count;
}

// Extracts real ORB descriptors from a real flight image via the actual ORBextractor (same
// config as eval_fast/config_212_golem27.yaml), so the "recorded frame" check exercises genuine
// image-derived descriptors rather than only uniform-random bit patterns. Returns an empty
// vector (soft-skip logged to stdout, not an error) if the image cannot be read.
std::vector<cv::Mat> ExtractRealFrameDescriptors(const std::string& image_path)
{
    const cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty())
    {
        std::cout << "vocabulary_converter --verify: real-frame image unavailable at " << image_path
                  << " - skipping the bonus real-frame check (random-descriptor check still runs)."
                  << std::endl;
        return {};
    }
    ORB_SLAM3::ORBextractor extractor(2500, 1.1F, 12, 5, 1);  // matches config_212_golem27.yaml
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> lapping_area{0, 0};  // monocular: ExtractORB(0, imGray, 0, 0) in Frame.cc
    extractor(image, cv::Mat(), keypoints, descriptors, lapping_area);

    std::vector<cv::Mat> feature_descriptors;
    feature_descriptors.reserve(static_cast<std::size_t>(descriptors.rows));
    for (int row = 0; row < descriptors.rows; ++row)
    {
        feature_descriptors.push_back(descriptors.row(row));
    }
    return feature_descriptors;
}

// Aggregate (whole-frame, one transform() call) + score() comparison, mirroring exactly how
// Frame::ComputeBoW/KeyFrame::ComputeBoW use the vocabulary in production.
bool VerifyAggregateAndScore(const StockVocabulary& text_vocabulary,
                              const ORB_SLAM3::CompactVocabulary& compact_vocabulary,
                              const std::vector<cv::Mat>& frame_a, const std::vector<cv::Mat>& frame_b)
{
    DBoW2::BowVector bow_text_a;
    DBoW2::FeatureVector feature_text_a;
    text_vocabulary.transform(frame_a, bow_text_a, feature_text_a, kLevelsUp);
    DBoW2::BowVector bow_compact_a;
    DBoW2::FeatureVector feature_compact_a;
    compact_vocabulary.Transform(frame_a, bow_compact_a, feature_compact_a, kLevelsUp);

    DBoW2::BowVector bow_text_b;
    DBoW2::FeatureVector feature_text_b;
    text_vocabulary.transform(frame_b, bow_text_b, feature_text_b, kLevelsUp);
    DBoW2::BowVector bow_compact_b;
    DBoW2::FeatureVector feature_compact_b;
    compact_vocabulary.Transform(frame_b, bow_compact_b, feature_compact_b, kLevelsUp);

    const bool aggregate_ok = BowVectorsBitExact(bow_text_a, bow_compact_a) &&
                              FeatureVectorsExact(feature_text_a, feature_compact_a) &&
                              BowVectorsBitExact(bow_text_b, bow_compact_b) &&
                              FeatureVectorsExact(feature_text_b, feature_compact_b);

    const double score_text = text_vocabulary.score(bow_text_a, bow_text_b);
    const double score_compact = compact_vocabulary.Score(bow_compact_a, bow_compact_b);
    const bool score_ok = std::memcmp(&score_text, &score_compact, sizeof(double)) == 0;

    std::cout << "vocabulary_converter --verify: real-frame aggregate bit-exact=" << aggregate_ok
              << " score bit-exact=" << score_ok << " (text=" << score_text
              << " compact=" << score_compact << ")" << std::endl;
    return aggregate_ok && score_ok;
}

// Orchestrates the whole --verify pass: loads both vocabularies, runs the random-descriptor
// sweep, the bonus real-frame aggregate+score check, and prints a final PASS/FAIL summary.
bool RunVerify(const std::string& text_vocabulary_path, const std::string& compact_vocabulary_path,
               const std::string& frame_image_path)
{
    StockVocabulary text_vocabulary;
    if (!text_vocabulary.loadFromTextFile(text_vocabulary_path))
    {
        std::cerr << "vocabulary_converter --verify: failed to load " << text_vocabulary_path
                  << std::endl;
        return false;
    }
    ORB_SLAM3::CompactVocabulary compact_vocabulary;
    if (!compact_vocabulary.Load(compact_vocabulary_path))
    {
        std::cerr << "vocabulary_converter --verify: failed to load " << compact_vocabulary_path
                  << std::endl;
        return false;
    }
    if (text_vocabulary.size() != compact_vocabulary.Size())
    {
        std::cerr << "vocabulary_converter --verify: size() mismatch (text=" << text_vocabulary.size()
                  << " compact=" << compact_vocabulary.Size() << ")" << std::endl;
        return false;
    }

    const unsigned int match_count =
        VerifyRandomDescriptors(text_vocabulary, compact_vocabulary, kRandomSampleCount);
    std::cout << "vocabulary_converter --verify: random-descriptor check " << match_count << "/"
              << kRandomSampleCount << " bit-exact matches" << std::endl;
    const bool random_ok = (match_count == kRandomSampleCount);

    const std::vector<cv::Mat> frame_a = ExtractRealFrameDescriptors(frame_image_path);
    bool frame_ok = true;
    if (!frame_a.empty())
    {
        std::mt19937_64 generator(0xF00DULL);
        std::vector<cv::Mat> frame_b;
        for (unsigned int index = 0U; index < 500U; ++index)
        {
            frame_b.push_back(RandomDescriptor(generator));
        }
        frame_ok = VerifyAggregateAndScore(text_vocabulary, compact_vocabulary, frame_a, frame_b);
    }

    const bool overall_ok = random_ok && frame_ok;
    std::cout << "vocabulary_converter --verify: " << (overall_ok ? "PASS" : "FAIL") << std::endl;
    return overall_ok;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: vocabulary_converter <text_vocab.txt> <output.cvoc> [--verify] "
                     "[--frame-image <path>]"
                  << std::endl;
        return 1;
    }
    const std::string text_vocabulary_path = argv[1];
    const std::string compact_vocabulary_path = argv[2];
    bool run_verify = false;
    std::string frame_image_path = kDefaultFrameImage;
    for (int arg_index = 3; arg_index < argc; ++arg_index)
    {
        const std::string argument = argv[arg_index];
        if (argument == "--verify")
        {
            run_verify = true;
        }
        else if (argument == "--frame-image" && arg_index + 1 < argc)
        {
            frame_image_path = argv[++arg_index];
        }
    }

    std::cout << "vocabulary_converter: converting " << text_vocabulary_path << " -> "
              << compact_vocabulary_path << std::endl;
    if (!ORB_SLAM3::CompactVocabulary::ConvertTextToCompact(text_vocabulary_path, compact_vocabulary_path))
    {
        std::cerr << "vocabulary_converter: conversion FAILED" << std::endl;
        return 1;
    }
    std::cout << "vocabulary_converter: conversion OK" << std::endl;

    if (run_verify && !RunVerify(text_vocabulary_path, compact_vocabulary_path, frame_image_path))
    {
        return 1;
    }
    return 0;
}
