#ifndef PUZZLESOLVERDOUBLECOMP_H
#define PUZZLESOLVERDOUBLECOMP_H

#include "puzzlesolver.h"

class PuzzleSolverDoubleComp : public PuzzleSolver
{
public:
    PuzzleSolverDoubleComp(std::shared_ptr <PuzzleComparator> comparator);

private:
    void solveImpl(std::shared_ptr<std::vector<Puzzle> > puzzles) override;
};

#endif // PUZZLESOLVERDOUBLECOMP_H
