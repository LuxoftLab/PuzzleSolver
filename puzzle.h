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
    const std::vector <cv::Point>& getEdgePoints(PuzzleAlignment align) const;
    const std::vector <cv::Scalar>& getEdgeColos(PuzzleAlignment align) const;
    bool isStraightSide(PuzzleAlignment align) const;
private:
    void processPuzzle(const cv::Mat &img, PuzzleAlignment align);
    void splitBackground(std::vector <int> &matr, const cv::Mat &img);
    std::pair <cv::Point, cv::Point> findPuzzle(std::vector <int> &matr, const cv::Mat &img);
    void correctCorners(const std::vector <int> &matr, const cv::Mat &img, std::vector <cv::Point> &corners);
    cv::Scalar meanColor(Puzzle::PuzzleAlignment align, cv::Point p);
private:
    int mPuzzleId;
    std::vector <cv::Point> mCorners[4];
    std::vector <cv::Point> mEdgePoints[4];
    std::pair <cv::Point, cv::Point> mEdgeRect[4];
    std::vector <cv::Scalar> mEdgeColors[4];
    cv::Mat mPuzzleImg[4];
    static std::vector <cv::Point> sMoves;
};

#endif // PUZZLE_H
