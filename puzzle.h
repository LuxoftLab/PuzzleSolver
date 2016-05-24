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

    struct PuzzleData
    {
        int mPuzzleId;
        std::vector <cv::Point> mCorners;
        std::vector <cv::Point> mEdgePoints;
        std::pair <cv::Point, cv::Point> mEdgeRect;
        std::vector <cv::Scalar> mEdgeColors;
        cv::Mat mPuzzleImg;
    };

    Puzzle(int puzzle_id, cv::Mat img, std::vector<cv::Point> corners);
    Puzzle(PuzzleData *data);

    int getPuzzleId() const;
    const cv::Mat& getImage(PuzzleAlignment align) const;
    int getHeight() const;
    int getWidth() const;
    cv::Point getCorner(PuzzleAlignment align, int corner) const;
    const std::vector <cv::Point>& getEdgePoints(PuzzleAlignment align) const;
    const std::vector <cv::Scalar>& getEdgeColors(PuzzleAlignment align) const;
    std::pair <cv::Point, cv::Point> getEdgeRect(PuzzleAlignment align) const;
    bool isStraightSide(PuzzleAlignment align) const;

    PuzzleData getPuzzle(Puzzle::PuzzleAlignment align) const;
    PuzzleData rotatePuzzle(Puzzle::PuzzleAlignment align, double angle) const;
private:
    void processPuzzle(const cv::Mat &img, PuzzleAlignment align);
    void splitBackground(std::vector <int> &matr, const cv::Mat &img) const;
    std::pair <cv::Point, cv::Point> findPuzzle(std::vector <int> &matr, const cv::Mat &img) const;
    void correctCorners(const std::vector <int> &matr, const cv::Mat &img, std::vector <cv::Point> &corners) const;
    cv::Scalar meanColor(Puzzle::PuzzleAlignment align, cv::Point p);
private:
//    std::vector <cv::Point> mCorners[4];
//    std::vector <cv::Point> mEdgePoints[4];
//    std::pair <cv::Point, cv::Point> mEdgeRect[4];
//    std::vector <cv::Scalar> mEdgeColors[4];
//    cv::Mat mPuzzleImg[4];
    PuzzleData mData[4];
    static std::vector <cv::Point> sMoves;
};

#endif // PUZZLE_H
