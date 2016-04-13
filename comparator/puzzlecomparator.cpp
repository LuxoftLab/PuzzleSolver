#include "puzzlecomparator.h"

PuzzleComparator::PuzzleComparator()
{

}

PuzzleComparator::ll PuzzleComparator::compare(const Puzzle &p1, const Puzzle &p2, Puzzle::PuzzleAlignment side)
{
    return combine_result(compareImpl(p1, p2, side));
}

