#ifndef PUZZLESOLVERSIMPLE_H
#define PUZZLESOLVERSIMPLE_H

#include "puzzlesolver.h"

class PuzzleSolverSimple : public PuzzleSolver
{
public:
    PuzzleSolverSimple(std::shared_ptr<PuzzleComparator> comparator);

private:
    void solveImpl(std::shared_ptr<std::vector<Puzzle> > puzzles) override;
};

#endif // PUZZLESOLVERSIMPLE_H
