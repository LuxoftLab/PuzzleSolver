#include "puzzlesolvergreedy.h"


PuzzleSolverGreedy::PuzzleSolverGreedy(std::shared_ptr<PuzzleComparator> comparator)
    : PuzzleSolver(comparator)
{

}

void PuzzleSolverGreedy::solveImpl(std::shared_ptr<std::vector<Puzzle> > puzzles)
{
    struct comp_item
    {
        int a, b;
        Puzzle::PuzzleAlignment side;
        comp_item(int _a, int _b, Puzzle::PuzzleAlignment _side)
            : a(_a)
            , b(_b)
            , side(_side)
        {}
    };
    std::vector <comp_item> v;
    for(int i = 0; i < int(puzzles->size()); i++)
    {
        for(int j = 0; j < int(puzzles->size()); j++)
        {
            if(puzzles->at(i).getPuzzleId() == puzzles->at(j).getPuzzleId())
                continue;
            for(int k = 0; k < 4; k++)
            {
                if(puzzles->at(i).isStraightSide(Puzzle::PuzzleAlignment(k)))
                    continue;
                if(pComparator->compare(puzzles->at(i), puzzles->at(j), Puzzle::PuzzleAlignment(k)) != -1)
                    v.push_back(comp_item(i, j, Puzzle::PuzzleAlignment(k)));
            }
        }
    }
    std::sort(v.begin(), v.end(), [this, &puzzles](const comp_item &x, const comp_item &y){
        return pComparator->compare(puzzles->at(x.a), puzzles->at(x.b), x.side)
                < pComparator->compare(puzzles->at(y.a), puzzles->at(y.b), y.side);
    });
    for(int i = 0; i < int(v.size()); i++)
    {
        size_t side = size_t(v[i].side);
        size_t opposite_side = (side + 2) % 2;
        if(mGraph[v[i].a][side] == -1 && mGraph[v[i].b][opposite_side] == -1)
        {
            mGraph[v[i].a][side] = v[i].b;
            mGraph[v[i].b][opposite_side] = v[i].a;
        }
    }
}


