#ifndef PUZZLESOLVERGREEDY_H
#define PUZZLESOLVERGREEDY_H

#include "puzzlesolver.h"

class PuzzleSolverGreedy : public PuzzleSolver
{
public:
    PuzzleSolverGreedy(std::shared_ptr <PuzzleComparator> comparator);

private:
    void solveImpl(std::shared_ptr<std::vector<Puzzle> > puzzles) override;
};

#endif // PUZZLESOLVERGREEDY_H
