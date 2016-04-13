#ifndef PUZZLECOMPARATORFIRSTIMPL_H
#define PUZZLECOMPARATORFIRSTIMPL_H

#include "puzzlecomparator.h"

class PuzzleComparatorFirstImpl : public PuzzleComparator
{
public:
    PuzzleComparatorFirstImpl(int max_color_dist);
private:

    ll calcSquareDist(const cv::Point &a, const cv::Point &b);
    int maxColorDiff(const cv::Scalar &a, const cv::Scalar &b);

    std::pair <ll, ll> compareImpl(const Puzzle &p1,
                                   const Puzzle &p2,
                                   Puzzle::PuzzleAlignment side) override;
    ll combine_result(std::pair<ll, ll> result) override;

    int mMaxColorDist;
};

#endif // PUZZLECOMPARATORFIRSTIMPL_H
