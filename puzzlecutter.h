#ifndef PUZZLECUTTER_H
#define PUZZLECUTTER_H

#include <vector>
#include <string>
#include <queue>
#include <puzzle.h>
#include <memory>
#include "debug.h"
#include "geometry.h"
#include "puzzle.h"

//#define CUTTER_STEP_RESULT

class Puzzle;

class PuzzleCutter
{
    struct PuzzleData;

private:
    PuzzleCutter();
    PuzzleCutter(const PuzzleCutter &other)=delete;
    PuzzleCutter operator=(const PuzzleCutter &other)=delete;

public:
    static PuzzleCutter& getInstanse();
    std::unique_ptr<std::vector <Puzzle>> cutImage(const std::vector<std::string> &files);
    std::unique_ptr<std::vector <Puzzle>> fromFile(std::string file_path);
    void saveToFile(std::shared_ptr<std::vector <Puzzle>> puzzles, std::string file_path);
    void rotateImg(cv::Mat& src, double angle, cv::Mat& dst);

    void getBinaryMask(const cv::Mat &src, cv::Mat &dst);

private:
    void convertToEdges(const cv::Mat &src, cv::Mat &dst);
    void divideIntoComponents(const cv::Mat &src, cv::Mat &dst);
    void findPuzzles(const cv::Mat &src);
    void findFeaturePoints(const cv::Mat &src, PuzzleData &pdata, int feature_value_threshold);
    void findPillarLines(const cv::Mat &src, PuzzleData &pdata);
    bool isPillarLines(const cv::Mat &src, PuzzleData &pdata);

    double puzzleAngle(const PuzzleData &pdata);

private:
    cv::Mat img;
    std::vector <PuzzleData> mPuzzlesData;
    std::vector <cv::Point> mMoves;
    std::vector <std::pair <int, bool> > mMatr;

private:
    static int sBackgroundThreshold;
    static double sRectAngleEps;
    static int sFeatureWindowSize;
    static int sErosionSize;
    static double sFeatureCoef;
public:
    static void setBackgroundThreshold(int th);
    static void setRectAngleEps(double eps);
    static void setFeatureWindowSize(int sz);
    static void setErosionSize(int sz);
    static void setFeatureCoef(double cf);

private:
    struct PuzzleData
    {
        PuzzleData()
            : area(0)
        {}
        int puzzleNumber;
        cv::Point minPt;
        cv::Point maxPt;
        std::vector <cv::Point> features;
        std::vector <std::pair <cv::Point, cv::Point> > pillarLines;
        int area;

        int height()
        {
            return maxPt.x - minPt.x + 1;
        }

        int width()
        {
            return maxPt.y - minPt.y + 1;
        }
    };
};

#endif // PUZZLECUTTER_H
