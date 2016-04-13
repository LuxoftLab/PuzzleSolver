#include "puzzlesolversimple.h"

PuzzleSolverSimple::PuzzleSolverSimple(std::shared_ptr<PuzzleComparator> comparator)
    : PuzzleSolver(comparator)
{

}

void PuzzleSolverSimple::solveImpl(std::shared_ptr<std::vector<Puzzle> > puzzles)
{
    for(int i = 0; i < int(puzzles->size()); i++)
    {
        std::vector <long long> best(4, std::numeric_limits<long long>::max());
        const Puzzle &cur_puzzle = puzzles->at(i);

        for(int k = 0; k < 4; k++)
        {
            if(cur_puzzle.isStraightSide(Puzzle::PuzzleAlignment(k)))
                continue;
            for(int j = 0; j < int(puzzles->size()); j++)
            {
                if(cur_puzzle.getPuzzleId() == puzzles->at(j).getPuzzleId())
                    continue;
                long long comp = pComparator->compare(cur_puzzle,
                                                    puzzles->at(j),
                                                    Puzzle::PuzzleAlignment(k));
                if(comp != -1 && best[k] > comp)
                {
                    mGraph[i][k] = j;
                    best[k] = comp;
                }
            }
        }
    }
}

