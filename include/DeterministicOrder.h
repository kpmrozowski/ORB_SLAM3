/**
 * Deterministic ordering for pointer-keyed containers (endpoint-gap determinism campaign).
 *
 * ORB-SLAM3 iterates std::set / std::map keyed by KeyFrame* or MapPoint* throughout the
 * mapping and optimization code. The default comparator orders by POINTER VALUE, i.e. by
 * allocator layout - measured to be non-reproducible run-to-run even single-threaded with
 * ASLR disabled. Iteration order decides g2o vertex/edge insertion order (floating-point
 * summation order) and covisibility weight-tie resolution, so results differed run to run.
 *
 * Ordering by the monotonically assigned mnId instead makes iteration order a pure function
 * of the (deterministic) creation sequence. Semantics are unchanged - the containers hold
 * the same elements; only the visit order is pinned.
 */
#ifndef DETERMINISTIC_ORDER_H
#define DETERMINISTIC_ORDER_H

namespace ORB_SLAM3
{

struct IdLess
{
    template <class T>
    bool operator()(const T* left, const T* right) const
    {
        if (!left || !right)
        {
            return left < right;
        }
        return left->mnId < right->mnId;
    }
};

}  // namespace ORB_SLAM3

#endif  // DETERMINISTIC_ORDER_H
