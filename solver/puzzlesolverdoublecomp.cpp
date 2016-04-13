#include "puzzlesolverdoublecomp.h"

PuzzleSolverDoubleComp::PuzzleSolverDoubleComp(std::shared_ptr<PuzzleComparator> comparator)
    : PuzzleSolver(comparator)
{

}

void PuzzleSolverDoubleComp::solveImpl(std::shared_ptr<std::vector<Puzzle> > puzzles)
{
    std::vector <int> start_puzzles;
    for(int i = 0; i < int(puzzles->size()); i++)
    {
        if(puzzles->at(i).isStraightSide(Puzzle::PuzzleAlignment::TopSide) &&
                puzzles->at(i).isStraightSide(Puzzle::PuzzleAlignment::LeftSide))
        {
            start_puzzles.push_back(i);
        }
    }

    for(int start_puzzle : start_puzzles)
    {
        std::fill(mUsedFlag.begin(), mUsedFlag.end(), false);
        std::queue <int> q;
        q.push(start_puzzle);
        mUsedFlag[puzzles->at(start_puzzle).getPuzzleId()] = true;
        while(!q.empty())
        {
            int v = q.front();
            q.pop();

            int to;

            // go right
            to = mGraph[v][3];
            int top_pzl = mGraph[v][2] != -1 ? (mGraph[mGraph[v][2]][3] != -1 ? mGraph[mGraph[v][2]][3] : -1) : -1;
            if(to == -1 && !puzzles->at(v).isStraightSide(Puzzle::PuzzleAlignment::RightSide))
            {
                long long best = std::numeric_limits <long long>::max();
                int best_pzl = -1;
                for(int i = 0; i < int(puzzles->size()); i++)
                {
                    if(mUsedFlag[puzzles->at(i).getPuzzleId()] || mGraph[i][1] != -1)
                        continue;
                    long long cur_right = pComparator->compare(puzzles->at(v), puzzles->at(i),
                                                               Puzzle::PuzzleAlignment::RightSide);
                    if(cur_right == -1)
                        continue;
                    long long cur_top = top_pzl == -1 ? 1 : (pComparator->compare(puzzles->at(i), puzzles->at(top_pzl),
                                                                                  Puzzle::PuzzleAlignment::TopSide));
                    if(cur_top == -1)
                        continue;
                    if(cur_right * cur_top < best)
                    {
                        best = cur_right * cur_top;
                        best_pzl = i;
                    }
                }
                if(best_pzl != -1)
                {
                    mGraph[v][3] = best_pzl;
                    mGraph[best_pzl][1] = v;
                    if(top_pzl != -1)
                    {
                        mGraph[top_pzl][0] = best_pzl;
                        mGraph[best_pzl][2] = top_pzl;
                    }
                    mUsedFlag[puzzles->at(best_pzl).getPuzzleId()] = true;
                    q.push(best_pzl);
                }
            }

            // go down
            to = mGraph[v][0];
            int left_pzl = mGraph[v][1] != -1 ? (mGraph[mGraph[v][1]][0] != -1 ? mGraph[mGraph[v][1]][0] : -1) : -1;
            if(to == -1 && !puzzles->at(v).isStraightSide(Puzzle::PuzzleAlignment::BottomSide))
            {
                long long best = std::numeric_limits <long long>::max();
                int best_pzl = -1;
                for(int i = 0; i < int(puzzles->size()); i++)
                {
                    if(mUsedFlag[puzzles->at(i).getPuzzleId()] || mGraph[i][2] != -1)
                        continue;
                    long long cur_down = pComparator->compare(puzzles->at(v), puzzles->at(i),
                                                              Puzzle::PuzzleAlignment::BottomSide);
                    if(cur_down == -1)
                        continue;
                    long long cur_left = left_pzl == -1 ? 1 : (pComparator->compare(puzzles->at(i), puzzles->at(left_pzl),
                                                                                    Puzzle::PuzzleAlignment::LeftSide));
                    if(cur_left == -1)
                        continue;
                    if(cur_down * cur_left < best)
                    {
                        best = cur_down * cur_left;
                        best_pzl = i;
                    }
                }
                if(best_pzl != -1)
                {
                    mGraph[v][0] = best_pzl;
                    mGraph[best_pzl][2] = v;
                    if(left_pzl != -1)
                    {
                        mGraph[left_pzl][3] = best_pzl;
                        mGraph[best_pzl][1] = left_pzl;
                    }
                    q.push(best_pzl);
                    mUsedFlag[puzzles->at(best_pzl).getPuzzleId()] = true;
                }
            }
        }
    }
}

