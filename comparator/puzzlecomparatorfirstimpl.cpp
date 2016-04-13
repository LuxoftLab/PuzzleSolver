#include "puzzlecomparatorfirstimpl.h"

PuzzleComparatorFirstImpl::PuzzleComparatorFirstImpl(int max_color_dist)
    : mMaxColorDist(max_color_dist)
{

}

PuzzleComparator::ll PuzzleComparatorFirstImpl::calcSquareDist(const cv::Point &a, const cv::Point &b)
{
    return 1ll*(a.x - b.x) * (a.x - b.x) + 1ll*(a.y - b.y) * (a.y - b.y);
}

int PuzzleComparatorFirstImpl::maxColorDiff(const cv::Scalar &a, const cv::Scalar &b)
{
    int ret = 0;
    for(int i = 0; i < 3; i++)
        ret = std::max(ret, std::abs(int(a[i]) - int(b[i])));
    return ret;
}

std::pair<PuzzleComparator::ll, PuzzleComparator::ll>
PuzzleComparatorFirstImpl::compareImpl(const Puzzle &p1,
                                       const Puzzle &p2,
                                       Puzzle::PuzzleAlignment side)
{
    Puzzle::PuzzleAlignment opposite_side = Puzzle::PuzzleAlignment((size_t(side)+2)%4);
    Puzzle::PuzzleAlignment checked_sides[2] = {
        Puzzle::PuzzleAlignment((size_t(side)+1)%4),
        Puzzle::PuzzleAlignment((size_t(opposite_side)+1)%4)
    };
    if(p1.isStraightSide(side) ^ p2.isStraightSide(opposite_side))
        return std::make_pair(-1ll, 0);
    for(int i = 0; i < 2; i++)
        if(p1.isStraightSide(checked_sides[i]) ^ p2.isStraightSide(checked_sides[i]))
            return std::make_pair(-1ll, 0);

    const std::vector <cv::Point> &p1_edges = p1.getEdgePoints(side);
    const std::vector <cv::Point> &p2_edges = p2.getEdgePoints(opposite_side);
    const std::vector <cv::Scalar> &p1_colors = p1.getEdgeColos(side);
    const std::vector <cv::Scalar> &p2_colors = p2.getEdgeColos(opposite_side);

    ll squareDist = 0;
    ll colorDiff = 0;

    double step = double(p2_edges.size()) / p1_edges.size();
    for(int i = 0; i < int(p1_edges.size()); i++)
    {
        cv::Point pt1 = p1_edges[i] - p1_edges[0];
        cv::Point pt2 = p2_edges[int(i * step)] - p2_edges[0];
        squareDist += calcSquareDist(pt1, pt2);
        colorDiff += maxColorDiff(p1_colors[i], p2_colors[int(i * step)]) > mMaxColorDist;
    }
    return std::make_pair(squareDist, colorDiff);
}

PuzzleComparator::ll PuzzleComparatorFirstImpl::combine_result(
        std::pair<PuzzleComparator::ll, PuzzleComparator::ll> result)
{
    if(result.first == -1)
        return -1;
    return result.first + result.second * 100;
}

