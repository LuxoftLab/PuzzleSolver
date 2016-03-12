#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef PUZZLECONTROLLER_H
#define PUZZLECONTROLLER_H


class PuzzleController
{
public:
    PuzzleController();
    void generatePuzzles(cv::Mat img);
    void save();
};

#endif // PUZZLECONTROLLER_H
