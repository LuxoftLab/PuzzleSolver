#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types_c.h>

#include <iostream>
#include "debug.h"

#include "puzzlecutter.h"
#include "comparator/puzzlecomparatorfirstimpl.h"
#include "solver/puzzlesolvergreedy.h"
#include "solver/puzzlesolversimple.h"
#include "solver/puzzlesolverdoublecomp.h"

std::vector <std::string> files {
    // list of files here
};



int main()
{
    PuzzleCutter::setBackgroundThreshold(5);
    PuzzleCutter::setFeatureWindowSize(4);
    PuzzleCutter::setErosionSize(1);
    PuzzleCutter::setFeatureCoef(3.2);
    std::cout << "Processing...\n";
    std::shared_ptr <std::vector <Puzzle>> puzzleContainer = PuzzleCutter::getInstanse().cutImage(files);
    std::cout << "Done!\n";

    PuzzleSolverDoubleComp solver(std::shared_ptr <PuzzleComparator>(new PuzzleComparatorFirstImpl(40)));
    std::vector <cv::Mat> imgs = solver.solve(puzzleContainer);
    for(cv::Mat &m : imgs)
    {
        cv::imshow("result", m);
        cv::waitKey();
    }
    return 0;
}

