#ifndef PUZZLECOMPARATOR_H
#define PUZZLECOMPARATOR_H

#include "puzzle.h"

class PuzzleComparator
{
public:
    typedef long long ll;

    PuzzleComparator();

    ll compare(const Puzzle &p1, const Puzzle &p2, Puzzle::PuzzleAlignment side);
private:
    virtual ll combine_result(std::pair <ll, ll> result) = 0;
    virtual std::pair <ll, ll> compareImpl(const Puzzle &p1,
                                           const Puzzle &p2,
                                           Puzzle::PuzzleAlignment side) = 0;
};

#endif // PUZZLECOMPARATOR_H
