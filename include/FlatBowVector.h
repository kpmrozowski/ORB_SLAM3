/**
* This file is part of ORB-SLAM3
*
* Task P3b (memory reduction): a flat, sorted-by-WordId replacement for a KeyFrame's
* DBoW2::BowVector (a std::map<WordId,WordValue>). A std::map spends ~48B per red-black-tree node
* on three pointers + colour + the pair; a flat std::vector spends ~12B per (WordId,double) entry,
* so a ~1750-word KeyFrame BoW shrinks from ~84KB to ~21KB. Enabled per-run by ORB_MEM_FLATBOW=1
* (see KeyFrame::IsFlatBowEnabled()); default OFF keeps the stock std::map path byte-for-byte.
*
* Bit-identity contract: FlatL1Score() reproduces DBoW2::L1Scoring::score() bit-for-bit. Both
* operands hold the exact same (WordId, weight) pairs the vocabulary's transform() produced (the
* flat vector is copied verbatim from the map, no recomputation), sorted ascending by WordId. The
* two-pointer merge-walk visits the shared words in the same ascending-WordId order the map-based
* skip-walk does, so the floating-point accumulation order -- and therefore the result bits -- are
* identical. This replicates ONLY the L1_NORM / TF_IDF combination ORBvoc.txt uses; callers assert
* that before switching to this path (see KeyFrame::BuildFlatBow / ORBVocabulary::UsesL1TfIdfScoring).
*/

#ifndef ORB_SLAM3_FLATBOWVECTOR_H
#define ORB_SLAM3_FLATBOWVECTOR_H

#include <utility>
#include <vector>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

namespace ORB_SLAM3
{

// Flat, sorted-ascending-by-WordId bag-of-words vector. Same logical contents as a
// DBoW2::BowVector (std::map<WordId,WordValue>) but with contiguous storage.
using FlatBowVector = std::vector<std::pair<DBoW2::WordId, DBoW2::WordValue>>;

// Bit-identical replacement for DBoW2::L1Scoring::score(first, second) where both operands are
// flat vectors. `first` plays the role of v1 (vi) and `second` the role of v2 (wi), matching the
// positional meaning in ScoringObject.cpp, so callers must pass the two operands in the same order
// the stock mpVoc->score(v1, v2) call used.
double FlatL1Score(const FlatBowVector& first, const FlatBowVector& second);

// Mixed-operand overload for the relocalization path, where the query is a (transient) Frame that
// keeps its std::map BoW and only the KeyFrame side is flat. `first` (the map) is v1 (vi), `second`
// (the flat) is v2 (wi) -- again matching mpVoc->score(F->mBowVec, pKFi->mBowVec).
double FlatL1Score(const DBoW2::BowVector& first, const FlatBowVector& second);

}  // namespace ORB_SLAM3

#endif  // ORB_SLAM3_FLATBOWVECTOR_H
