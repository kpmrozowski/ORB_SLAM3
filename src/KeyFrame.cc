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

#include "DeterministicOrder.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "ImuTypes.h"
#include<mutex>

#include <cstdio>
#include <cstdlib>

#include <MemoryGovernor.h>

namespace ORB_SLAM3
{

namespace
{

// Task P1: swaps `container` with a freshly default-constructed (zero-capacity) instance,
// releasing any heap-allocated storage back to the allocator when the temporary is destroyed at
// the end of this call. Works uniformly for std::vector<T>, the nested KeyFrame grid type, and
// DBoW2's std::map-derived BowVector/FeatureVector (all expose a member swap()).
// Container::clear() alone is insufficient here: for std::vector it empties the logical contents
// but retains the allocated capacity, so RSS would not actually drop.
template <typename ContainerType>
void SwapWithEmpty(ContainerType& container)
{
    ContainerType empty_container;
    container.swap(empty_container);
}

// ORB_MEM_PARANOIA: aborts with the offending KeyFrame's id and the accessor name that triggered
// the read. Deliberately a hard abort (not an exception) -- this is a validation-only tool for a
// dedicated flight, not a production error path.
void AbortOnReleasedPayloadRead(const long unsigned int keyframe_id, const char* const accessor_name)
{
    std::fprintf(stderr,
                 "ORB_MEM_PARANOIA: read of released payload on bad KeyFrame id=%lu via %s\n",
                 keyframe_id, accessor_name);
    std::abort();
}

// Task P3c: true only for a KeyFrame source Frame that provably carries no stereo/RGBD data, i.e.
// the monocular(-inertial) case where mvKeys/mvuRight/mvDepth are dead weight. Guards the drop so
// enabling ORB_MEM_DROP_MONO_DEADFIELDS on a stereo/fisheye/RGBD run is a safe no-op:
//  - Fisheye stereo (NLeft != -1) reads mvKeys/mvKeysRight in GetFeaturesInArea and the culling /
//    observation-level octave lookups, so it must keep the fields.
//  - Rectified-stereo / RGBD (NLeft == -1) carries real depth, surfaced as non-negative mvuRight
//    (mvuRight[i] is the right-image x-coordinate, positive whenever mvDepth[i] > 0); any such entry
//    means UnprojectStereo()/GetKpURight() have live data to read.
// A pure monocular frame has NLeft == -1 and every mvuRight entry == -1, so the fields are dead.
bool MonoFrameHasNoStereoData(const Frame& source_frame)
{
    if (source_frame.Nleft != -1)
    {
        return false;
    }
    for (const float right_coord : source_frame.mvuRight)
    {
        if (right_coord >= 0.0f)
        {
            return false;
        }
    }
    return true;
}

}  // namespace

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame():
        mnFrameId(0),  mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(0), mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)),
        mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0), mvInvLevelSigma2(0), mnMinX(0), mnMinY(0), mnMaxX(0),
        mnMaxY(0), mPrevKF(static_cast<KeyFrame*>(NULL)), mNextKF(static_cast<KeyFrame*>(NULL)),
        mvKeysUnData(static_cast<vector<cv::KeyPoint> >(NULL)), mvuRight(static_cast<vector<float> >(NULL)), mvDepth(static_cast<vector<float> >(NULL)),
        mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mnMergeCorrectedForKF(0),
        NLeft(0),NRight(0), mnNumberOfOpt(0), mbHasVelocity(false)
{

}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys),
    mBowVec(F.mBowVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK_(F.mK_), mPrevKF(NULL), mNextKF(NULL), mpImuPreintegrated(F.mpImuPreintegrated),
    mImuCalib(F.mImuCalib),
    mvKeysUnData(F.mvKeysUn), mDescriptorsData(F.mDescriptors.clone()), mFeatVecData(F.mFeatVec),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth),
    mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mnMergeCorrectedForKF(0),
    mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
    mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch), mTlr(F.GetRelativePoseTlr()),
    mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.GetRelativePoseTrl()), mnNumberOfOpt(0), mbHasVelocity(false)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }



    if(!F.HasVelocity()) {
        mVw.setZero();
        mbHasVelocity = false;
    }
    else
    {
        mVw = F.GetVelocity();
        mbHasVelocity = true;
    }

    mImuBias = F.mImuBias;
    SetPose(F.GetPose());

    mnOriginMapId = pMap->GetId();

    // Task P3c (memory reduction): drop the mono-dead payload. The init list above copied mvKeys/
    // mvuRight/mvDepth from the Frame exactly as stock (so the OFF path is byte-for-byte unchanged);
    // here, under ORB_MEM_DROP_MONO_DEADFIELDS and only for a provably-mono source frame, we release
    // those copies right away so no live monocular KeyFrame keeps them resident. Steady-state memory
    // is identical to never allocating (each vector is emptied before the KeyFrame is used); the
    // transient is one frame's worth, freed inside the ctor. See IsDropMonoDeadFieldsEnabled() and
    // the accessor comments for the proof that these fields are dead on the mono path.
    if (IsDropMonoDeadFieldsEnabled() && MonoFrameHasNoStereoData(F))
    {
        SwapWithEmpty(mvKeys);
        SwapWithEmpty(mvuRight);
        SwapWithEmpty(mvDepth);
    }
}

// Task P3b: ORB_MEM_FLATBOW knob, read once (env is fixed for the process lifetime). Unset or 0 =>
// stock std::map BoW path, byte-for-byte; 1 => build the flat vector and free the map in ComputeBoW.
bool KeyFrame::IsFlatBowEnabled()
{
    static const char* const value = getenv("ORB_MEM_FLATBOW");
    static const bool enabled = value != nullptr && atoi(value) != 0;
    return enabled;
}

// Task P3c: ORB_MEM_DROP_MONO_DEADFIELDS knob, read once (env is fixed for the process lifetime).
// Unset or 0 => keep mvKeys/mvuRight/mvDepth on every KeyFrame, byte-for-byte stock behaviour.
bool KeyFrame::IsDropMonoDeadFieldsEnabled()
{
    static const char* const value = getenv("ORB_MEM_DROP_MONO_DEADFIELDS");
    static const bool enabled = value != nullptr && atoi(value) != 0;
    return enabled;
}

void KeyFrame::ComputeBoW()
{
    if (MemoryGovernor::ParanoiaEnabled() && mbPayloadReleased)
    {
        AbortOnReleasedPayloadRead(mnId, "ComputeBoW");
    }

    // Defensive fault-in for symmetry with the GetKeysUn()/GetDescriptorsMat()/GetFeatVec()
    // accessors: ComputeBoW() reads the spillable mDescriptorsData/mFeatVecData directly below. Its
    // only call sites (LocalMapping/Tracking) run on a freshly-created, resident KeyFrame, so this
    // is a no-op today; it hardens the path should ComputeBoW() ever be reached on an evicted KF.
    EnsureResident();

    // Task P3b: once the flat BoW is built the std::map mBowVec is freed, so the stock
    // empty()-guarded recompute below must not re-fire on a later ComputeBoW() call (the two
    // initialization KeyFrames are computed once in Tracking and again in LocalMapping). A non-empty
    // flat vector is the "already built" signal; KeyFrames always carry >0 words in practice.
    const bool flat_bow_enabled = IsFlatBowEnabled();
    if (flat_bow_enabled && !mBowVecFlat.empty())
    {
        return;
    }

    if(mBowVec.empty() || mFeatVecData.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptorsData);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVecData,4);
    }

    if (flat_bow_enabled)
    {
        // Only the L1_NORM/TF_IDF combination is replicated bit-for-bit by FlatL1Score; abort loudly
        // rather than silently diverge if a different vocabulary is ever configured.
        if (!mpORBvocabulary->UsesL1TfIdfScoring())
        {
            std::fprintf(stderr,
                         "ORB_MEM_FLATBOW=1 requires an L1_NORM/TF_IDF vocabulary; aborting.\n");
            std::abort();
        }

        // Copy the transform() doubles verbatim; std::map iterates ascending by WordId, so the flat
        // vector comes out sorted -- exactly what FlatL1Score's merge-walk requires for bit-identity.
        mBowVecFlat.clear();
        mBowVecFlat.reserve(mBowVec.size());
        for (DBoW2::BowVector::const_iterator word_it = mBowVec.begin(), word_end = mBowVec.end();
             word_it != word_end; ++word_it)
        {
            mBowVecFlat.emplace_back(word_it->first, word_it->second);
        }

        // std::map::clear() leaves the red-black-tree nodes allocated; swapping with an empty map
        // returns them to the allocator so RSS actually drops (same idiom as SwapWithEmpty above).
        DBoW2::BowVector().swap(mBowVec);
    }
}

void KeyFrame::SetPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexPose);

    mTcw = Tcw;
    mRcw = mTcw.rotationMatrix();
    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();

    if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
    }
}

void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw)
{
    unique_lock<mutex> lock(mMutexPose);
    mVw = Vw;
    mbHasVelocity = true;
}

Sophus::SE3f KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}

Sophus::SE3f KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

Eigen::Vector3f KeyFrame::GetCameraCenter(){
    unique_lock<mutex> lock(mMutexPose);
    return mTwc.translation();
}

Eigen::Vector3f KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return mOwb;
}

Eigen::Matrix3f KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Sophus::SE3f KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc * mImuCalib.mTcb;
}

Eigen::Matrix3f KeyFrame::GetRotation(){
    unique_lock<mutex> lock(mMutexPose);
    return mRcw;
}

Eigen::Vector3f KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw.translation();
}

Eigen::Vector3f KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return mVw;
}

bool KeyFrame::isVelocitySet()
{
    unique_lock<mutex> lock(mMutexPose);
    return mbHasVelocity;
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(), vPairs.end(),
         [](const pair<int,KeyFrame*>& left, const pair<int,KeyFrame*>& right)
         {
             if (left.first != right.first)
             {
                 return left.first < right.first;
             }
             return left.second->mnId < right.second->mnId;  // deterministic tie-break (was pointer order)
         });
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
    {
        return vector<KeyFrame*>();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<KeyFrame*>();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(this);
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1)
        mvpMapPoints[leftIndex]=static_cast<MapPoint*>(NULL);
    if(rightIndex != -1)
        mvpMapPoints[rightIndex]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::NullMapPointSlotsIn(const std::unordered_set<MapPoint*>& doomed)
{
    unique_lock<mutex> lock(mMutexFeatures);
    for(size_t slot_index=0, slot_end=mvpMapPoints.size(); slot_index<slot_end; ++slot_index)
    {
        MapPoint* const slot_map_point = mvpMapPoints[slot_index];
        if(slot_map_point!=nullptr && doomed.find(slot_map_point)!=doomed.end())
        {
            mvpMapPoints[slot_index]=nullptr;
        }
    }
}

set<MapPoint*, IdLess> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*, IdLess> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent)
{
    map<KeyFrame*,int,IdLess> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,tuple<int,int>,IdLess> observations = pMP->GetObservations();

        for(map<KeyFrame*,tuple<int,int>,IdLess>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                continue;
            KFcounter[mit->first]++;

        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(), vPairs.end(),
         [](const pair<int,KeyFrame*>& left, const pair<int,KeyFrame*>& right)
         {
             if (left.first != right.first)
             {
                 return left.first < right.first;
             }
             return left.second->mnId < right.second->mnId;  // deterministic tie-break (was pointer order)
         });
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());


        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    if(pKF == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*, IdLess> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*, IdLess> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::AddMergeEdge(KeyFrame* pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspMergeEdges.insert(pKF);
}

set<KeyFrame*, IdLess> KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspMergeEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==mpMap->GetInitKFid())
        {
            return;
        }
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(this);
    }

    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            mvpMapPoints[i]->EraseObservation(this);
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            mpParent->EraseChild(this);
            mTcp = mTcw * mpParent->GetPoseInverse();
        }
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);

    // Task P1-fix (memory reduction, UAF fix): by this point `this` is unlinked from the Map,
    // the KeyFrameDatabase inverted index, the covisibility graph and every MapPoint's
    // observation list (all above). Release the heavy payload INLINE, right here, rather than
    // enqueueing it for a later MemoryGovernor::Tick(): the original deferred design assumed
    // KeyFrames are never delete()d, which is false -- LocalMapping::InitializeIMU() and
    // ::ScaleRefinement() both run `(*lit)->SetBadFlag(); delete *lit;` back to back on entries
    // of mlNewKeyFrames (src/LocalMapping.cc), reachable every frame from
    // SpinOnceDeterministic(). A queued `this` could therefore be freed before the deferred
    // Tick() dereferenced it -- a use-after-free. Calling ReleaseBadPayload() here, before
    // SetBadFlag() returns, closes that window: nothing can delete `this` out from under a
    // pending release. Safe with respect to the fields released here (mDescriptors/mBowVec/
    // mFeatVec/mGrid/mvKeys/mvDepth): every reader of them is either isBad()-guarded or
    // unreachable in monocular mode, verified by the required 4-cell determinism gate -- see
    // task-P1-fix-report.md. mvKeysUn/mvuRight/mvpMapPoints remain KEPT (not released), unchanged
    // from Task P1 -- see ReleaseBadPayload(). Deterministic-mode + env-gated; a single
    // static-bool check otherwise.
    if (MemoryGovernor::ReclaimBadPayloadEnabled())
    {
        ReleaseBadPayload();
    }
}

void KeyFrame::ReleaseBadPayload()
{
    if (mbPayloadReleased)
    {
        return;
    }

    // Task P3d: when the spill governor is active a background SpillWorker may be serializing this
    // KeyFrame's mDescriptorsData/mFeatVecData/mGrid (under mMutexFeatures). Take that same mutex
    // around the frees below so a bad-KF release can never race the worker's read. Deferred-lock so
    // the reclaim-only (budget-off) path is byte-for-byte unchanged from Task P1. mvKeysUnData is
    // still KEPT here (P1) -- an evicted-then-bad KeyFrame keeps a valid on-disk record, so a stale
    // observation reader faults its keysUn back to identical bytes via the accessor.
    std::unique_lock<std::mutex> features_lock(mMutexFeatures, std::defer_lock);
    if (MemoryGovernor::SpillActive())
    {
        features_lock.lock();
    }

    // Partial release (Task P1). KEPT deliberately -- mvKeysUnData, mvuRight, mvpMapPoints:
    // stock ORB-SLAM3 leaves STALE OBSERVATIONS behind (CreateNewMapPoints can overwrite a
    // neighbour-KF slot via AddMapPoint without erasing the overwritten MapPoint's observation
    // -- traced live: KF35 slot 87 held MP12960, overwritten by MP12962; MP12960 kept
    // (KF35,87) in mObservations for good). Such live MapPoints later make LOAD-BEARING
    // reads/writes into this bad KF's arrays: KeyFrameCulling's observer loop reads
    // GetKeysUn()[idx].octave with no isBad() guard (the value feeds the redundancy count),
    // MapPoint::SetBadFlag/Replace write mvpMapPoints slots via EraseMapPointMatch/
    // ReplaceMapPointMatch, and MapPoint::EraseObservation reads mvuRight[idx] for its nObs
    // accounting.
    //
    // Task P3a investigated releasing these three (routing every reader through the accessors
    // above is exactly what was needed to make the compiler enumerate them), and specifically
    // tried adding the missing `if(pKFi->isBad()) continue;` guard at the KeyFrameCulling site.
    // That guard was empirically PROVEN to change the OFF-mode (no-reclaim) trajectory on the
    // required fast-gate flight (212_golem27, NF=2500): isolated via systematic-debugging by
    // disabling just that one guard, which alone restored the canonical f_/kf_ md5. I.e. stock
    // ORB-SLAM3 genuinely reaches this stale-observation path and depends on reading the bad
    // KF's (still-present, unreleased) keypoint data on this flight -- exactly the case the task
    // brief said to leave unguarded rather than ship. Per that brief's explicit fallback, the
    // guard and the release of these three fields are NOT shipped; P3a ships only the accessor
    // refactor (all external reads routed through GetKeysUn()/GetKpURight()/GetKpDepth()/
    // GetMapPoint() etc.), unchanged in what it releases from Task P1. See task-P3a-report.md
    // for the full investigation and the isolation-test evidence.
    //
    // Everything below (mvKeys/mvDepth/mGrid/mBowVec/mFeatVecData/mDescriptorsData) has NO
    // stale-observation reader: remaining readers are isBad()-guarded (ComputeDistinctiveDescriptors,
    // LocalBA/GBA edge builders, culling candidates) or live-/NotErase-scoped (matchers, KFDB
    // after erase), verified per-site in task-P1-report.md.
    SwapWithEmpty(mvKeys);
    SwapWithEmpty(mvDepth);
    SwapWithEmpty(mGrid);
    SwapWithEmpty(mBowVec);
    // Task P3b: under ORB_MEM_FLATBOW the map above is already empty (freed in ComputeBoW) and this
    // flat vector is the resident BoW; free it too so a bad KeyFrame drops its whole BoW footprint.
    // KeyFrameDatabase::erase() runs before this (SetBadFlag erases from the DB, then releases), so
    // the inverted-file cleanup has already read the flat words it needs.
    SwapWithEmpty(mBowVecFlat);
    SwapWithEmpty(mFeatVecData);

    if (MemoryGovernor::ParanoiaEnabled())
    {
        // A released cv::Mat is 0x0 (rows==0), which several size-driven loops elsewhere treat as
        // "nothing to do" and silently skip -- unhelpful for validation. Use a recognizable
        // non-empty sentinel instead, so any direct read of mDescriptorsData (via
        // GetDescriptorsMat()) that bypasses the ComputeBoW()/GetFeaturesInArea() guards above
        // surfaces as an obviously wrong 1x1 matrix rather than a quietly-skipped empty one.
        mDescriptorsData = cv::Mat(1, 1, CV_8UC1, cv::Scalar(0xAA));
    }
    else
    {
        mDescriptorsData.release();
    }

    mbPayloadReleased = true;
    MemoryGovernor::IncrementKfShellReleased();
}

// Task P3a: fault-in choke points. Pure indirection over the still-present, privatized members --
// zero behavior change versus the direct field reads these replace across ~65 external call
// sites. Bounds-checking these against release (so P3c/P3d can later actually drop/page the
// underlying vectors) is deliberately left to that later task, not added speculatively here.
const std::vector<cv::KeyPoint>& KeyFrame::GetKeysUn()
{
    EnsureResident();
    return mvKeysUnData;
}

const cv::Mat& KeyFrame::GetDescriptorsMat()
{
    EnsureResident();
    return mDescriptorsData;
}

const DBoW2::FeatureVector& KeyFrame::GetFeatVec()
{
    EnsureResident();
    return mFeatVecData;
}

float KeyFrame::GetKpURight(const int keypoint_index) const
{
    // Task P3c: an empty backing vector means the mono-dead fields were dropped (or this is the
    // serialization default ctor); synthesize the -1 monocular sentinel the vector would have held.
    if (mvuRight.empty())
    {
        return -1.0f;
    }
    return mvuRight[keypoint_index];
}

float KeyFrame::GetKpDepth(const int keypoint_index) const
{
    // Task P3c: see GetKpURight() -- an empty mvDepth reads back the -1 monocular sentinel.
    if (mvDepth.empty())
    {
        return -1.0f;
    }
    return mvDepth[keypoint_index];
}

// Task P3d: fault-in choke point. One acquire-load fast path when the payload is hot (this is the
// entire cost when ORB_MEM_BUDGET_MB is unset -- residency never leaves kResident). On a cold miss
// the governor's SpillWorker restores identical bytes (const_cast is safe: the observable const
// state -- the payload values -- is unchanged, only its residency in RAM).
void KeyFrame::EnsureResident() const
{
    const std::uint8_t state = mSpillResidency.load(std::memory_order_acquire);
    if (spill::PayloadPresent(state))
    {
        return;
    }
    MemoryGovernor::Instance().FaultIn(const_cast<KeyFrame*>(this));
}

// Worker thread: read-only copy of the immutable payload into a spill body, under mMutexFeatures so
// it cannot race a concurrent ReleaseBadPayload() free. Returns false (skip write) if the KeyFrame
// went bad / released its payload before we serialized it (there is then no valid record to write;
// its kept keysUn stays resident).
bool KeyFrame::SpillSerialize(std::vector<std::uint8_t>& out_body)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if (mbPayloadReleased || mDescriptorsData.empty())
    {
        return false;
    }
    spill::SerializePayloadBody(mDescriptorsData, mvKeysUnData, mFeatVecData, out_body);
    return true;
}

// Worker thread (or a WaitResident caller that stole a queued load): install a faulted-in payload
// and rebuild the (never-spilled) grid, under mMutexFeatures. Idempotency is keyed on the residency
// ATOMIC, not on mDescriptorsData: the caller reaches here only after winning the exclusive CAS to
// kLoading (see SpillWorker::WaitResident / WorkerLoop, which check PayloadPresent() before the
// CAS), so exactly one agent installs per fault-in and no double-install can happen. Keying the
// guard on !mDescriptorsData.empty() would misfire under ORB_MEM_PARANOIA=1: ReleaseBadPayload()
// leaves a non-empty 1x1 0xAA poison sentinel in mDescriptorsData, which would short-circuit the
// install and leave mvKeysUnData empty -> OOB on the next GetKeysUn(). Install unconditionally.
void KeyFrame::SpillInstall(const std::vector<std::uint8_t>& body)
{
    unique_lock<mutex> lock(mMutexFeatures);
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keys_undistorted;
    DBoW2::FeatureVector feature_vector;
    if (!spill::DeserializePayloadBody(body.data(), body.size(), N, descriptors, keys_undistorted,
                                       feature_vector))
    {
        std::fprintf(stderr, "KeyFrame %lu: corrupt spill payload on fault-in\n",
                     static_cast<unsigned long>(mnId));
        std::abort();
    }
    mDescriptorsData = descriptors;
    mvKeysUnData = std::move(keys_undistorted);
    mFeatVecData = std::move(feature_vector);

    // Rebuild mGrid exactly as Frame::AssignFeaturesToGrid did for the monocular source frame
    // (ascending i over mvKeysUnData, identical PosInGrid rounding) -> byte-identical grid, so
    // GetFeaturesInArea returns the same indices in the same order it did before eviction.
    const int reserve_per_cell = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
    mGrid.resize(mnGridCols);
    for (int cell_x = 0; cell_x < mnGridCols; ++cell_x)
    {
        mGrid[cell_x].assign(mnGridRows, std::vector<std::size_t>());
        for (int cell_y = 0; cell_y < mnGridRows; ++cell_y)
        {
            mGrid[cell_x][cell_y].reserve(reserve_per_cell);
        }
    }
    for (int feature_index = 0; feature_index < N; ++feature_index)
    {
        const cv::KeyPoint& keypoint = mvKeysUnData[feature_index];
        const int pos_x = round((keypoint.pt.x - mnMinX) * mfGridElementWidthInv);
        const int pos_y = round((keypoint.pt.y - mnMinY) * mfGridElementHeightInv);
        if (pos_x >= 0 && pos_x < FRAME_GRID_COLS && pos_y >= 0 && pos_y < FRAME_GRID_ROWS)
        {
            mGrid[pos_x][pos_y].push_back(feature_index);
        }
    }
}

// Main thread, at the deterministic governor tick only: free the spilled payload + drop the grid.
// Never called for a KeyFrame in a worker-owned state (kWriteQueued/kLoading), so it cannot race
// the worker; guarded by mMutexFeatures for the store-store publication of the emptied fields.
void KeyFrame::SpillFree()
{
    unique_lock<mutex> lock(mMutexFeatures);
    SwapWithEmpty(mvKeysUnData);
    SwapWithEmpty(mFeatVecData);
    SwapWithEmpty(mGrid);
    mDescriptorsData.release();
}

// Eligible for spill/eviction: has a real feature payload, is alive and has not P1-released.
bool KeyFrame::SpillEligible()
{
    if (N <= 0)
    {
        return false;
    }
    if (isBad())
    {
        return false;
    }
    unique_lock<mutex> lock(mMutexFeatures);
    return !mbPayloadReleased;
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    // Task P3d: GetFeaturesInArea reads mGrid + mvKeysUnData directly (both spilled/dropped on
    // eviction), so it is a fault-in point too -- restore the payload and rebuild the grid first.
    EnsureResident();
    if (MemoryGovernor::ParanoiaEnabled() && mbPayloadReleased)
    {
        AbortOnReleasedPayloadRead(mnId, "GetFeaturesInArea");
    }
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUnData[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
{
    // Task P3c: a mono KeyFrame drops mvDepth/mvKeys (all -1); treat the empty vector as depth -1 so
    // this returns false without indexing released storage. Mono never reaches here with z>0 anyway
    // -- callers gate on bStereo = (!mpCamera2 && GetKpURight()>=0), which is false in mono.
    const float z = mvDepth.empty() ? -1.0f : mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        x3D = mRwc * x3Dc + mTwc.translation();
        return true;
    }
    else
        return false;
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    if(N==0)
        return -1.0;

    vector<MapPoint*> vpMapPoints;
    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        tcw = mTcw.translation();
        Rcw = mRcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float,1,3> Rcw2 = Rcw.row(2);
    float zcw = tcw(2);
    for(int i=0; i<N; i++) {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            Eigen::Vector3f x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

Eigen::Vector3f KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

Eigen::Vector3f KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void KeyFrame::PreSave(set<KeyFrame*, IdLess>& spKF,set<MapPoint*, IdLess>& spMP, set<GeometricCamera*>& spCam)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the vector
    mvBackupMapPointsId.clear();
    mvBackupMapPointsId.reserve(N);
    for(int i = 0; i < N; ++i)
    {

        if(mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end()) // Checks if the element is not null
            mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
        else // If the element is null his value is -1 because all the id are positives
            mvBackupMapPointsId.push_back(-1);
    }
    // Save the id of each connected KF with it weight
    mBackupConnectedKeyFrameIdWeights.clear();
    for(std::map<KeyFrame*,int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it)
    {
        if(spKF.find(it->first) != spKF.end())
            mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }

    // Save the parent id
    mBackupParentId = -1;
    if(mpParent && spKF.find(mpParent) != spKF.end())
        mBackupParentId = mpParent->mnId;

    // Save the id of the childrens KF
    mvBackupChildrensId.clear();
    mvBackupChildrensId.reserve(mspChildrens.size());
    for(KeyFrame* pKFi : mspChildrens)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupChildrensId.push_back(pKFi->mnId);
    }

    // Save the id of the loop edge KF
    mvBackupLoopEdgesId.clear();
    mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
    for(KeyFrame* pKFi : mspLoopEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupLoopEdgesId.push_back(pKFi->mnId);
    }

    // Save the id of the merge edge KF
    mvBackupMergeEdgesId.clear();
    mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
    for(KeyFrame* pKFi : mspMergeEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupMergeEdgesId.push_back(pKFi->mnId);
    }

    //Camera data
    mnBackupIdCamera = -1;
    if(mpCamera && spCam.find(mpCamera) != spCam.end())
        mnBackupIdCamera = mpCamera->GetId();

    mnBackupIdCamera2 = -1;
    if(mpCamera2 && spCam.find(mpCamera2) != spCam.end())
        mnBackupIdCamera2 = mpCamera2->GetId();

    //Inertial data
    mBackupPrevKFId = -1;
    if(mPrevKF && spKF.find(mPrevKF) != spKF.end())
        mBackupPrevKFId = mPrevKF->mnId;

    mBackupNextKFId = -1;
    if(mNextKF && spKF.find(mNextKF) != spKF.end())
        mBackupNextKFId = mNextKF->mnId;

    if(mpImuPreintegrated)
        mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
}

void KeyFrame::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId){
    // Rebuild the empty variables

    // Pose
    SetPose(mTcw);

    mTrl = mTlr.inverse();

    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mvpMapPoints.clear();
    mvpMapPoints.resize(N);
    for(int i=0; i<N; ++i)
    {
        if(mvBackupMapPointsId[i] != -1)
            mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
        else
            mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
    }

    // Conected KeyFrames with him weight
    mConnectedKeyFrameWeights.clear();
    for(map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end();
        it != end; ++it)
    {
        KeyFrame* pKFi = mpKFid[it->first];
        mConnectedKeyFrameWeights[pKFi] = it->second;
    }

    // Restore parent KeyFrame
    if(mBackupParentId>=0)
        mpParent = mpKFid[mBackupParentId];

    // KeyFrame childrens
    mspChildrens.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it!=end; ++it)
    {
        mspChildrens.insert(mpKFid[*it]);
    }

    // Loop edge KeyFrame
    mspLoopEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it)
    {
        mspLoopEdges.insert(mpKFid[*it]);
    }

    // Merge edge KeyFrame
    mspMergeEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupMergeEdgesId.begin(), end = mvBackupMergeEdgesId.end(); it != end; ++it)
    {
        mspMergeEdges.insert(mpKFid[*it]);
    }

    //Camera data
    if(mnBackupIdCamera >= 0)
    {
        mpCamera = mpCamId[mnBackupIdCamera];
    }
    else
    {
        cout << "ERROR: There is not a main camera in KF " << mnId << endl;
    }
    if(mnBackupIdCamera2 >= 0)
    {
        mpCamera2 = mpCamId[mnBackupIdCamera2];
    }

    //Inertial data
    if(mBackupPrevKFId != -1)
    {
        mPrevKF = mpKFid[mBackupPrevKFId];
    }
    if(mBackupNextKFId != -1)
    {
        mNextKF = mpKFid[mBackupNextKFId];
    }
    mpImuPreintegrated = &mBackupImuPreintegrated;


    // Remove all backup container
    mvBackupMapPointsId.clear();
    mBackupConnectedKeyFrameIdWeights.clear();
    mvBackupChildrensId.clear();
    mvBackupLoopEdgesId.clear();

    UpdateBestCovisibles();
}

bool KeyFrame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY = Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY= Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u = fx * PcX * invz + cx;
    v = fy * PcY * invz + cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

Sophus::SE3f KeyFrame::GetRelativePoseTrl()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTrl;
}

Sophus::SE3f KeyFrame::GetRelativePoseTlr()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTlr;
}

Sophus::SE3<float> KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    return mTrl * mTcw;
}

Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);

    return mTwc * mTlr;
}

Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTwc * mTlr).translation();
}

Eigen::Matrix<float,3,3> KeyFrame::GetRightRotation() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTrl.so3() * mTcw.so3()).matrix();
}

Eigen::Vector3f KeyFrame::GetRightTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    return (mTrl * mTcw).translation();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

} //namespace ORB_SLAM
