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
#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include <MemoryGovernor.h>

namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint():
    mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL))
{
    mpReplaced = static_cast<MapPoint*>(NULL);
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    SetWorldPos(Pos);

    mNormalVector.setZero();

    mbTrackInViewR = false;
    mbTrackInView = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    mInvDepth=invDepth;
    mInitU=(double)uv_init.x;
    mInitV=(double)uv_init.y;
    mpHostKF = pHostKF;

    mNormalVector.setZero();

    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
{
    SetWorldPos(Pos);

    Eigen::Vector3f Ow;
    if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        Eigen::Matrix3f Rwl = pFrame->GetRwc();
        Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->GetOw();

        Ow = Rwl * tlr + twl;
    }
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / mNormalVector.norm();

    Eigen::Vector3f PC = mWorldPos - Ow;
    const float dist = PC.norm();
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos = Pos;
}

Eigen::Vector3f MapPoint::GetWorldPos() {
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos;
}

Eigen::Vector3f MapPoint::GetNormal() {
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector;
}


KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

bool MapPoint::observation_key_less(const ObservationEntry& entry, const long unsigned int key_id)
{
    return entry.first->mnId < key_id;
}

MapPoint::ObservationVector::const_iterator MapPoint::find_observation(
    const ObservationVector& observations, KeyFrame* const pKF)
{
    const long unsigned int key_id = pKF->mnId;
    const ObservationVector::const_iterator lower = std::lower_bound(
        observations.begin(), observations.end(), key_id, &MapPoint::observation_key_less);
    if(lower != observations.end() && lower->first->mnId == key_id)
    {
        return lower;
    }
    return observations.end();
}

MapPoint::ObservationVector::iterator MapPoint::lower_bound_observation(KeyFrame* const pKF)
{
    return std::lower_bound(
        mObservations.begin(), mObservations.end(), pKF->mnId, &MapPoint::observation_key_less);
}

void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    const ObservationVector::iterator lower = lower_bound_observation(pKF);
    const bool found = (lower != mObservations.end() && lower->first->mnId == pKF->mnId);

    tuple<int,int> indexes;
    if(found){
        indexes = lower->second;
    }
    else{
        indexes = tuple<int,int>(-1,-1);
    }

    if(pKF -> NLeft != -1 && idx >= pKF -> NLeft){
        get<1>(indexes) = idx;
    }
    else{
        get<0>(indexes) = idx;
    }

    if(found){
        // Same-mnId key already present: replace the payload only, keeping the stored KeyFrame*
        // exactly as std::map::operator[] did (the key is never rebound on an existing entry).
        lower->second = indexes;
    }
    else{
        // Insert at the lower-bound position, keeping mObservations sorted ascending by mnId.
        mObservations.insert(lower, ObservationEntry(pKF, indexes));
    }

    if(!pKF->mpCamera2 && pKF->GetKpURight(idx)>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        const ObservationVector::iterator it = lower_bound_observation(pKF);
        if(it != mObservations.end() && it->first->mnId == pKF->mnId)
        {
            tuple<int,int> indexes = it->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                if(!pKF->mpCamera2 && pKF->GetKpURight(leftIndex)>=0)
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

            mObservations.erase(it);

            // Reproduces the old `mpRefKF = mObservations.begin()->first`: the smallest-mnId
            // remaining observer, which for the sorted vector is front(). Guarded against the
            // empty case, where the old std::map dereferenced end() (undefined behaviour whose
            // garbage result was immediately discarded because nObs<=2 sets the bad flag below and
            // SetBadFlag never reads mpRefKF) — the guard leaves mpRefKF==pKF, equally unread, so
            // the deterministic output is unchanged while a vector-front() dereference crash is
            // avoided.
            if(mpRefKF==pKF && !mObservations.empty())
            {
                mpRefKF=mObservations.front().first;
            }

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}


std::map<KeyFrame*, std::tuple<int,int>, IdLess>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    // Rebuild the historical return type from the flat vector. mObservations is sorted ascending
    // by mnId and IdLess orders ascending by mnId, so emplace_hint(end(), ...) inserts in strictly
    // increasing key order — an O(n) rebuild that yields a map iterating in the exact former order.
    std::map<KeyFrame*, std::tuple<int,int>, IdLess> observations;
    for(const ObservationEntry& entry : mObservations)
    {
        observations.emplace_hint(observations.end(), entry.first, entry.second);
    }
    return observations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    ObservationVector obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(const ObservationEntry& observation : obs)
    {
        KeyFrame* const pKF = observation.first;
        const int leftIndex = get<0>(observation.second), rightIndex = get<1>(observation.second);
        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapPointMatch(rightIndex);
        }
    }

    mpMap->EraseMapPoint(this);

    // Task P1 (memory reduction): defer the mDescriptor release to MemoryGovernor::Tick()
    // (one-KF-tick delay). It CANNOT be released here: TrackWithMotionModel still matches
    // against this (now bad) MapPoint's descriptor during the next frame, via the
    // mLastFrame.mvpMapPoints pointer chain — SearchLocalPoints only scrubs bad MapPoints from
    // the chain at that frame's TrackLocalMap. Immediate release was proven to SIGSEGV in
    // ORBmatcher::DescriptorDistance 15s into the fast gate. Deterministic-mode + env-gated;
    // a single static-bool-pair check otherwise.
    if (MemoryGovernor::ReclaimBadPayloadEnabled())
    {
        MemoryGovernor::Instance().DeferMapPointRelease(this);
    }

    // Task P6 (memory reduction): quarantine-delete this now-bad MapPoint. Every observer's
    // mvpMapPoints slot was nulled above (EraseMapPointMatch over the complete mObservations set)
    // and it has been erased from the Map, so after a K-KF-tick quarantine -- long enough for the
    // last/current Frame's mvpMapPoints and the mpReplaced chain read by
    // Tracking::CheckReplacedInLastFrame, and LocalMapping::mlpRecentAddedMapPoints, to have
    // cycled it out -- MemoryGovernor::Tick() delete()s it (see MemoryGovernor.h for the full
    // pointer/container coverage proof). mbDeleteQueued makes this strictly once-only. Fully
    // inert (a single static-bool check) unless ORB_MEM_DELETE_QUARANTINE>0 in deterministic mode.
    if (!mbDeleteQueued && MemoryGovernor::DeleteQuarantineActive())
    {
        mbDeleteQueued = true;
        MemoryGovernor::Instance().DeferMapPointDelete(this);
    }
}

void MapPoint::ReleaseBadDescriptor()
{
    // Called by MemoryGovernor::Tick() one tick after SetBadFlag() (never directly from SLAM
    // code). mDescriptor.clone() in ComputeDistinctiveDescriptors()/GetDescriptor() means the
    // Mat buffer is exclusively owned by this MapPoint (no aliasing), so release() frees it.
    unique_lock<mutex> lock(mMutexFeatures);
    if (mbDescriptorReleased)
    {
        return;
    }
    mDescriptor.release();
    mbDescriptorReleased = true;
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    ObservationVector obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(const ObservationEntry& observation : obs)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = observation.first;

        tuple<int,int> indexes = observation.second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(!pMP->IsInKeyFrame(pKF))
        {
            if(leftIndex != -1){
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            if(leftIndex != -1){
                pKF->EraseMapPointMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);

    // Task P6 (memory reduction): Replace() is the second path that baddens a MapPoint (mbBad set
    // above), leaking `this` exactly like SetBadFlag() does. Quarantine-delete it too. `this`
    // keeps mpReplaced=pMP so Tracking::CheckReplacedInLastFrame can still resolve a last-Frame
    // slot for one more frame; the K-KF-tick quarantine outlives that read, and the deletion order
    // is FIFO by baddening time, so `this` (enqueued now) is always delete()d strictly before its
    // still-live replacement pMP is ever enqueued -- no dangling mpReplaced is ever read. See
    // MemoryGovernor.h. Once-only via mbDeleteQueued; inert unless ORB_MEM_DELETE_QUARANTINE>0.
    if (!mbDeleteQueued && MemoryGovernor::DeleteQuarantineActive())
    {
        mbDeleteQueued = true;
        MemoryGovernor::Instance().DeferMapPointDelete(this);
    }
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);

    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    ObservationVector observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(const ObservationEntry& observation : observations)
    {
        KeyFrame* pKF = observation.first;

        if(!pKF->isBad()){
            tuple<int,int> indexes = observation.second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                vDescriptors.push_back(pKF->GetDescriptorsMat().row(leftIndex));
            }
            if(rightIndex != -1){
                vDescriptors.push_back(pKF->GetDescriptorsMat().row(rightIndex));
            }
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    // ORB_MEM_PARANOIA (Task P1, validation-only): any descriptor read after
    // ReleaseBadDescriptor() would silently produce an empty Mat where stock code reads real
    // bytes (a determinism bug, and the exact class of read behind the pre-deferral SIGSEGV in
    // ORBmatcher::DescriptorDistance) — abort loudly with the MapPoint id instead.
    if (MemoryGovernor::ParanoiaEnabled() && mbDescriptorReleased)
    {
        std::fprintf(stderr,
                     "ORB_MEM_PARANOIA: GetDescriptor() on released bad MapPoint id=%lu\n",
                     mnId);
        std::abort();
    }
    return mDescriptor.clone();
}

tuple<int,int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    const ObservationVector::const_iterator it = find_observation(mObservations, pKF);
    if(it != mObservations.end())
    {
        return it->second;
    }
    else
    {
        return tuple<int,int>(-1,-1);
    }
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return find_observation(mObservations, pKF) != mObservations.end();
}

void MapPoint::UpdateNormalAndDepth()
{
    ObservationVector observations;
    KeyFrame* pRefKF;
    Eigen::Vector3f Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos;
    }

    if(observations.empty())
        return;

    Eigen::Vector3f normal;
    normal.setZero();
    int n=0;
    for(const ObservationEntry& observation : observations)
    {
        KeyFrame* pKF = observation.first;

        tuple<int,int> indexes = observation.second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(leftIndex != -1){
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
        if(rightIndex != -1){
            Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
    const float dist = PC.norm();

    // Formerly observations[pRefKF]: pRefKF is always one of the observers, so this resolves to its
    // stored (left,right) indexes. The absent-key fallback mirrors std::map::operator[], which
    // would have value-initialized a (0,0) entry.
    const ObservationVector::const_iterator ref_it = find_observation(observations, pRefKF);
    tuple<int ,int> indexes = (ref_it != observations.end()) ? ref_it->second : tuple<int,int>();
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int level;
    if(pRefKF -> NLeft == -1){
        level = pRefKF->GetKeysUn()[leftIndex].octave;
    }
    else if(leftIndex != -1){
        level = pRefKF -> mvKeys[leftIndex].octave;
    }
    else{
        level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
    }

    //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

void MapPoint::SetNormalVector(const Eigen::Vector3f& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

void MapPoint::PrintObservations()
{
    cout << "MP_OBS: MP " << mnId << endl;
    for(const ObservationEntry& observation : mObservations)
    {
        KeyFrame* pKFi = observation.first;
        tuple<int,int> indexes = observation.second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
    }
}

Map* MapPoint::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void MapPoint::PreSave(set<KeyFrame*, IdLess>& spKF,set<MapPoint*, IdLess>& spMP)
{
    mBackupReplacedId = -1;
    if(mpReplaced && spMP.find(mpReplaced) != spMP.end())
        mBackupReplacedId = mpReplaced->mnId;

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
    // Save the id and position in each KF who views it. EraseObservation() below mutates
    // mObservations (and can trigger SetBadFlag, which clears it), so iterate over a snapshot in
    // the same ascending-mnId order the former std::map produced rather than mutating the live
    // container mid-walk (which the old code did through the map's stable-node erase semantics — a
    // pattern a std::vector cannot reproduce safely). Erasing an already-absent KF is a no-op, so
    // once a SetBadFlag cascade empties mObservations the remaining EraseObservation calls simply
    // do nothing, exactly as the map version's post-clear count()==0 checks did.
    const ObservationVector observations_snapshot = mObservations;
    for(const ObservationEntry& observation : observations_snapshot)
    {
        KeyFrame* const pKFi = observation.first;
        if(spKF.find(pKFi) != spKF.end())
        {
            mBackupObservationsId1[pKFi->mnId] = get<0>(observation.second);
            mBackupObservationsId2[pKFi->mnId] = get<1>(observation.second);
        }
        else
        {
            EraseObservation(pKFi);
        }
    }

    // Save the id of the reference KF
    if(spKF.find(mpRefKF) != spKF.end())
    {
        mBackupRefKFId = mpRefKF->mnId;
    }
}

void MapPoint::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid)
{
    mpRefKF = mpKFid[mBackupRefKFId];
    if(!mpRefKF)
    {
        cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
    }
    mpReplaced = static_cast<MapPoint*>(NULL);
    if(mBackupReplacedId>=0)
    {
        map<long unsigned int, MapPoint*>::iterator it = mpMPid.find(mBackupReplacedId);
        if (it != mpMPid.end())
            mpReplaced = it->second;
    }

    mObservations.clear();

    // mBackupObservationsId1 is a std::map keyed by KeyFrame mnId, so it iterates in ascending
    // mnId order, and mpKFid[it->first] resolves the KeyFrame whose mnId == it->first. Appending
    // the resolved observers in that order therefore builds mObservations already sorted ascending
    // by mnId — the invariant every lookup/insert path relies on — with no post-sort needed.
    for(map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
    {
        KeyFrame* pKFi = mpKFid[it->first];
        map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
        std::tuple<int, int> indexes = tuple<int,int>(it->second,it2->second);
        if(pKFi)
        {
           mObservations.emplace_back(pKFi, indexes);
        }
    }

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
}

} //namespace ORB_SLAM
