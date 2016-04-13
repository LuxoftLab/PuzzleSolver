#ifndef PUZZLESOLVER_H
#define PUZZLESOLVER_H

#include "comparator/puzzlecomparator.h"

//#define SOLVER_STEP_RESULT

class PuzzleSolver
{
public:
    PuzzleSolver(std::shared_ptr<PuzzleComparator> comparator);
    std::vector <cv::Mat> solve(std::shared_ptr <std::vector <Puzzle>> puzzles);

private:
    virtual void solveImpl(std::shared_ptr <std::vector <Puzzle>> puzzles) = 0;
    cv::Mat combineResult(std::shared_ptr <std::vector <Puzzle>> puzzles, int start_puzzle);
protected:
    std::shared_ptr <PuzzleComparator> pComparator;
    std::vector <std::vector <int> > mGraph;
    std::vector <bool> mUsedFlag;
};

#endif // PUZZLESOLVER_H
