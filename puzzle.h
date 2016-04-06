#ifndef PUZZLE_H
#define PUZZLE_H

#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <queue>
#include <stack>
#include <functional>
#include "debug.h"
#include "geometry.h"
#include "puzzlecutter.h"

class Puzzle
{
public:
    enum class PuzzleAlignment
    {
        BottomSide,
        LeftSide,
        TopSide,
        RightSide,
    };

    Puzzle(int puzzle_id, cv::Mat img, std::vector<cv::Point> corners);

    int getPuzzleId() const;
    const cv::Mat& getImage(PuzzleAlignment align) const;
    int getHeight() const;
    int getWidth() const;
    cv::Point getCorner(PuzzleAlignment align, int corner) const;
private:
    void processPuzzle(const cv::Mat &img, PuzzleAlignment align);
    void correctCorners(const std::vector <int> &matr, const cv::Mat &img, PuzzleAlignment align);
private:
    int mPuzzleId;
    //cv::Point mMinPt;
    //cv::Point mMaxPt;
    //std::vector <std::pair<cv::Point, cv::Point> > mPillarLines;
    std::vector <cv::Point> mCorners[4];
    //std::vector <cv::Point> mEdgeTopPoints;
    //std::vector <cv::Point> mEdgeBottomPoints;
    cv::Mat mPuzzleImg[4];
};

#endif // PUZZLE_H
