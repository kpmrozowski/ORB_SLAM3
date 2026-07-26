/**
* This file is part of ORB-SLAM3
*
* Task P3b (memory reduction): flat BowVector L1 scoring. See include/FlatBowVector.h for the
* bit-identity contract. Both functions below are transcriptions of DBoW2::L1Scoring::score()
* (Thirdparty/DBoW2/DBoW2/ScoringObject.cpp): the same skip-walk structure, the same accumulation
* expression `fabs(vi - wi) - fabs(vi) - fabs(wi)`, and the same final `score = -score/2.0`, so
* that on identical (WordId, weight) inputs they return byte-identical doubles.
*/

#include "FlatBowVector.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ORB_SLAM3
{

namespace
{

// Ordering predicate for std::lower_bound over a FlatBowVector: an entry precedes `word_id` when
// its WordId is smaller. Mirrors std::map::lower_bound(word_id), which the stock skip-walk uses to
// jump the trailing operand forward to the first element with key >= word_id.
bool word_id_less(const std::pair<DBoW2::WordId, DBoW2::WordValue>& entry, const DBoW2::WordId word_id)
{
    return entry.first < word_id;
}

}  // namespace

double FlatL1Score(const FlatBowVector& first, const FlatBowVector& second)
{
    FlatBowVector::const_iterator first_it = first.begin();
    FlatBowVector::const_iterator second_it = second.begin();
    const FlatBowVector::const_iterator first_end = first.end();
    const FlatBowVector::const_iterator second_end = second.end();

    double score = 0;

    while (first_it != first_end && second_it != second_end)
    {
        const DBoW2::WordValue& vi = first_it->second;
        const DBoW2::WordValue& wi = second_it->second;

        if (first_it->first == second_it->first)
        {
            score += std::fabs(vi - wi) - std::fabs(vi) - std::fabs(wi);

            ++first_it;
            ++second_it;
        }
        else if (first_it->first < second_it->first)
        {
            first_it = std::lower_bound(first_it, first_end, second_it->first, word_id_less);
        }
        else
        {
            second_it = std::lower_bound(second_it, second_end, first_it->first, word_id_less);
        }
    }

    score = -score / 2.0;

    return score;
}

double FlatL1Score(const DBoW2::BowVector& first, const FlatBowVector& second)
{
    DBoW2::BowVector::const_iterator first_it = first.begin();
    FlatBowVector::const_iterator second_it = second.begin();
    const DBoW2::BowVector::const_iterator first_end = first.end();
    const FlatBowVector::const_iterator second_end = second.end();

    double score = 0;

    while (first_it != first_end && second_it != second_end)
    {
        const DBoW2::WordValue& vi = first_it->second;
        const DBoW2::WordValue& wi = second_it->second;

        if (first_it->first == second_it->first)
        {
            score += std::fabs(vi - wi) - std::fabs(vi) - std::fabs(wi);

            ++first_it;
            ++second_it;
        }
        else if (first_it->first < second_it->first)
        {
            first_it = first.lower_bound(second_it->first);
        }
        else
        {
            second_it = std::lower_bound(second_it, second_end, first_it->first, word_id_less);
        }
    }

    score = -score / 2.0;

    return score;
}

}  // namespace ORB_SLAM3
