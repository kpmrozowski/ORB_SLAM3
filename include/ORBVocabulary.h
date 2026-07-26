/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include <memory>
#include <string>
#include <vector>

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

#include "CompactVocabulary.h"

namespace ORB_SLAM3
{

// The stock DBoW2 template, kept under its own name (rather than only as an anonymous base of
// ORBVocabulary below) so the converter/introspector and --verify tooling can still name the
// concrete DBoW2 type directly.
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> DBoW2Vocabulary;

// Thin dispatch wrapper (Task P2). Before this task ORBVocabulary was a typedef for
// DBoW2Vocabulary; it is now a concrete class holding EITHER a DBoW2Vocabulary (stock text path)
// OR a mmap'd CompactVocabulary (ORB_VOC_COMPACT=1 path), selected at load time by which of
// loadFromTextFile()/LoadCompact() is called. Every existing call site (System/Tracking/
// LoopClosing/KeyFrameDatabase/Frame/KeyFrame) keeps using `ORBVocabulary*`/`ORBVocabulary&`
// exactly as before - only the four methods ORB-SLAM3 actually calls on a vocabulary object
// (grep-verified across the src/ directory: loadFromTextFile, transform, score, size) are
// forwarded here.
// Everything else DBoW2::TemplatedVocabulary offers (vocabulary *creation*, cv::FileStorage
// save/load, stopWords, ...) is unused by ORB-SLAM3 and intentionally not exposed.
class ORBVocabulary
{
public:
    ORBVocabulary() = default;

    // Stock path (ORB_VOC_COMPACT unset): parses ORBvoc.txt exactly as before this task.
    bool loadFromTextFile(const std::string& text_vocabulary_path)
    {
        mCompactMode = false;
        return mTextVocabulary.loadFromTextFile(text_vocabulary_path);
    }

    // Compact path (ORB_VOC_COMPACT=1): mmaps an already-built .cvoc file. System.cc owns the
    // "build it if missing" decision and the one-time log line (see LoadOrbVocabulary()); this
    // method only ever mmaps a path that is expected to already exist.
    bool LoadCompact(const std::string& compact_vocabulary_path)
    {
        mCompactMode = true;
        mpCompactVocabulary = std::make_unique<CompactVocabulary>();
        return mpCompactVocabulary->Load(compact_vocabulary_path);
    }

    void transform(const std::vector<DBoW2::FORB::TDescriptor>& features, DBoW2::BowVector& bow_vector,
                   DBoW2::FeatureVector& feature_vector, const int levelsup) const
    {
        if (mCompactMode)
        {
            mpCompactVocabulary->Transform(features, bow_vector, feature_vector, levelsup);
        }
        else
        {
            mTextVocabulary.transform(features, bow_vector, feature_vector, levelsup);
        }
    }

    double score(const DBoW2::BowVector& bow_vector_a, const DBoW2::BowVector& bow_vector_b) const
    {
        return mCompactMode ? mpCompactVocabulary->Score(bow_vector_a, bow_vector_b)
                             : mTextVocabulary.score(bow_vector_a, bow_vector_b);
    }

    unsigned int size() const
    {
        return mCompactMode ? mpCompactVocabulary->Size() : mTextVocabulary.size();
    }

    // Task P3b: the flat-BowVector scoring path (ORB_MEM_FLATBOW=1) hand-rolls DBoW2::L1Scoring
    // and therefore only reproduces the L1_NORM / TF_IDF combination. Callers assert this before
    // switching to that path. The compact path is L1_NORM/TF_IDF by construction --
    // CompactVocabulary::Load() rejects any .cvoc that is not (see CompactVocabulary.cc), and the
    // class holds a DBoW2::L1Scoring object unconditionally -- so it always answers true when in
    // compact mode; the text path is queried directly.
    bool UsesL1TfIdfScoring() const
    {
        if (mCompactMode)
        {
            return true;
        }
        return mTextVocabulary.getScoringType() == DBoW2::L1_NORM
            && mTextVocabulary.getWeightingType() == DBoW2::TF_IDF;
    }

private:
    bool mCompactMode = false;
    DBoW2Vocabulary mTextVocabulary;
    std::unique_ptr<CompactVocabulary> mpCompactVocabulary;
};

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
