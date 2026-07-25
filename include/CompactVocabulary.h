/**
 * CompactVocabulary — Task P2: read-only mmap'd sidecar for the ORB vocabulary tree.
 *
 * Replaces DBoW2's ~401MB text-parsed TemplatedVocabulary (1.08M `Node`s, each with a
 * heap-allocated `cv::Mat` descriptor, plus a 10M-slot `m_words` reserve) with a ~61MB
 * Structure-of-Arrays blob (see the `.cvoc` header comment in CompactVocabulary.cc)
 * that is mapped `PROT_READ, MAP_SHARED` and never copied into the heap. Only the four
 * DBoW2Vocabulary members ORB-SLAM3 actually calls (grep-verified across the src/ directory:
 * loadFromTextFile, transform, score, size) are reproduced; everything DBoW2 offers for
 * *building* a vocabulary (HKmeansStep, create, ...) is intentionally absent.
 *
 * Bit-exactness contract: Transform()/Score() must return byte-identical BowVector/
 * FeatureVector/double results to DBoW2Vocabulary::transform()/score() on the same
 * descriptors, for every descriptor, forever (the trajectory md5 determinism gate depends
 * on this). This is proven empirically by Examples/Vocabulary/vocabulary_converter.cc
 * --verify, not merely by code inspection. This format is deliberately narrow: it only
 * supports L1_NORM scoring / TF_IDF weighting (the only combination ORBvoc.txt uses) and
 * asserts that at both build time (ConvertTextToCompact) and load time (Load).
 *
 * ORB_VOC_COMPACT=1 (see ORBVocabulary::LoadCompact, src/System.cc) is the only caller of
 * this class in the running system; the converter tool is the only other caller.
 */

#ifndef COMPACTVOCABULARY_H
#define COMPACTVOCABULARY_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/ScoringObject.h"

namespace ORB_SLAM3
{

class CompactVocabulary
{
public:
    CompactVocabulary() = default;
    ~CompactVocabulary();

    CompactVocabulary(const CompactVocabulary&) = delete;
    CompactVocabulary& operator=(const CompactVocabulary&) = delete;

    // mmaps (PROT_READ, MAP_SHARED) an existing .cvoc file written by ConvertTextToCompact().
    // Validates the magic, the payload CRC64, and that scoring/weighting match the only
    // combination this format supports (L1_NORM/TF_IDF). Returns false (logging one line to
    // stderr) on any failure; the object stays unloaded (Size()==0, Transform() a no-op).
    bool Load(const std::string& compact_vocabulary_path);

    // Parses <text_vocabulary_path> with DBoW2's own TemplatedVocabulary::loadFromTextFile
    // (via a private-member-introspecting subclass, so the .cvoc arrays are copied verbatim
    // from DBoW2's own parsed tree rather than from an independent re-implementation of the
    // text grammar) and writes <compact_vocabulary_path> atomically (temp file + rename).
    // Shared by both the standalone converter tool and System.cc's auto-build-if-missing path.
    static bool ConvertTextToCompact(const std::string& text_vocabulary_path,
                                      const std::string& compact_vocabulary_path);

    // Bit-exact port of TemplatedVocabulary::transform(features, v, fv, levelsup)
    // (Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h:1126-1194 for the aggregate loop,
    // :1218-1259 for the per-descriptor child descent) against the mmap'd arrays. A no-op
    // (clears v/fv) if Load() has not succeeded.
    void Transform(const std::vector<DBoW2::FORB::TDescriptor>& features, DBoW2::BowVector& bow_vector,
                   DBoW2::FeatureVector& feature_vector, const int levelsup) const;

    // Delegates to DBoW2::L1Scoring::score - the only scoring type this format supports, so
    // there is nothing else to dispatch on. Pure function of the two BowVectors; works
    // identically regardless of which vocabulary (text or compact) produced them.
    double Score(const DBoW2::BowVector& bow_vector_a, const DBoW2::BowVector& bow_vector_b) const;

    // Number of words (n_words header field). 0 if not loaded.
    unsigned int Size() const { return mNumWords; }

    bool IsLoaded() const { return mMappedBase != nullptr; }

private:
    void TransformOne(const DBoW2::FORB::TDescriptor& feature, DBoW2::WordId& word_id,
                       DBoW2::WordValue& weight, DBoW2::NodeId* const node_id_out,
                       const int levelsup) const;
    cv::Mat DescriptorAt(const std::uint32_t node_id) const;
    void Unmap();

    void* mMappedBase = nullptr;
    std::size_t mMappedLength = 0;

    std::uint32_t mBranchingFactor = 0;  // k (unused by transform/score, kept for introspection)
    std::uint32_t mDepthLevels = 0;      // L
    std::uint32_t mNumNodes = 0;
    std::uint32_t mNumWords = 0;

    // Views into the mmap'd region; valid only while mMappedBase != nullptr.
    const double* mWeight = nullptr;
    const std::uint32_t* mChildFirst = nullptr;
    const std::uint32_t* mChildCount = nullptr;
    const std::uint32_t* mWordId = nullptr;
    const unsigned char* mDescriptorBytes = nullptr;
    const std::uint32_t* mChildIdx = nullptr;

    DBoW2::L1Scoring mScoringObject;
};

}  // namespace ORB_SLAM3

#endif  // COMPACTVOCABULARY_H
