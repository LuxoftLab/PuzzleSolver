#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "constheader.h"

#ifndef GENERATOR_H
#define GENERATOR_H

class PuzzleGenerator
{
public:
    PuzzleGenerator();
    void generate(cv::Mat img);
private:
    void rotatePuzzle(const cv::Mat & src, cv::Mat & dst);
    void drawPuzzle(cv::Mat mask,
                    int koefWidth,
                    int koefHeight,
                    int i, int j,
                    int sign, int sign2,
                    int angle);
    void setPuzzleForm(cv::Mat mask);
    void setMask(cv::Mat img);
    void makeSimplePuzzle(cv::Mat imgOriginal);
    void floodFillPuzzle(cv::Mat &puzzle, cv::Point pxl, int cnt, cv::Point center);
    int shufflePuzzles();
    void cutImageOnNParts();
};

#endif // GENERATOR_H

