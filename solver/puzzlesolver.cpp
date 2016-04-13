#include "puzzlesolver.h"

PuzzleSolver::PuzzleSolver(std::shared_ptr<PuzzleComparator> comparator)
    : pComparator(comparator)
{

}

std::vector<cv::Mat> PuzzleSolver::solve(std::shared_ptr<std::vector<Puzzle> > puzzles)
{
    LOG << "Solving start...\n";

    mGraph.resize(puzzles->size());
    for(int i = 0; i < int(mGraph.size()); i++)
        mGraph[i].resize(4, -1);
    mUsedFlag.resize(puzzles->size(), false);

    solveImpl(puzzles);

    std::vector <cv::Mat> ret;
    for(int i = 0; i < int(puzzles->size()); i++)
    {
        if(puzzles->at(i).isStraightSide(Puzzle::PuzzleAlignment::TopSide) &&
                puzzles->at(i).isStraightSide(Puzzle::PuzzleAlignment::LeftSide))
            ret.push_back(combineResult(puzzles, i));
    }

    mGraph.clear();
    mUsedFlag.clear();

    LOG << "Solving end!\n";

    return ret;
}

cv::Mat PuzzleSolver::combineResult(std::shared_ptr<std::vector<Puzzle> > puzzles, int start_puzzle)
{
    cv::Mat m(7000, 7000, puzzles->at(start_puzzle).getImage(Puzzle::PuzzleAlignment::BottomSide).type(),
              cv::Scalar(0, 0, 0));
    std::fill(mUsedFlag.begin(), mUsedFlag.end(), false);
    std::queue <std::pair <int, cv::Point> > q;
    q.push({start_puzzle, cv::Point(15, 15)});
    mUsedFlag[puzzles->at(start_puzzle).getPuzzleId()] = true;
    cv::Point max_pt(0, 0);
    while(!q.empty())
    {
        int v = q.front().first;
        cv::Point p = q.front().second;
        q.pop();
        Puzzle &pzl = puzzles->at(v);

        cv::Mat img = pzl.getImage(Puzzle::PuzzleAlignment::TopSide);
        cv::Point mns = pzl.getCorner(Puzzle::PuzzleAlignment::TopSide, 2);
        for(int i = 0; i < img.rows; i++)
        {
            for(int j = 0; j < img.cols; j++)
            {
                if(img.at <cv::Vec3b> (i, j) != cv::Vec3b(0,0,0))
                {
                    m.at <cv::Vec3b> (i + p.y- mns.y, j + p.x - mns.x) = img.at <cv::Vec3b> (i, j);
                    max_pt.x = std::max(max_pt.x, j + p.x - mns.x);
                    max_pt.y = std::max(max_pt.y, i + p.y - mns.y);
                }
            }
        }
#ifdef SOLVER_STEP_RESULT
        cv::imshow("step", m);
        cv::waitKey();
#endif

        int to;

        // go right
        to = mGraph[v][3];
        if(to != -1 && !mUsedFlag[puzzles->at(to).getPuzzleId()])
        {
            cv::Point pto(p-pzl.getCorner(Puzzle::PuzzleAlignment::TopSide, 2) +
                          pzl.getCorner(Puzzle::PuzzleAlignment::TopSide, 3));
            mUsedFlag[puzzles->at(to).getPuzzleId()] = true;
            q.push({to, pto});
        }

        //go down
        to = mGraph[v][0];
        if(to != -1 && !mUsedFlag[puzzles->at(to).getPuzzleId()])
        {
           cv::Point pto(p-pzl.getCorner(Puzzle::PuzzleAlignment::TopSide, 2) +
                         pzl.getCorner(Puzzle::PuzzleAlignment::TopSide, 0));
           mUsedFlag[puzzles->at(to).getPuzzleId()] = true;
           q.push({to, pto});
        }
    }
    cv::Rect rct(cv::Point(0, 0), max_pt + cv::Point(15, 15));
    return m(rct);
}

