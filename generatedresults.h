#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>




#ifndef GENERATEDRESULTS_H
#define GENERATEDRESULTS_H


class GeneratedResults
{
public:
    std::vector <cv::Mat> generatedPuzzles;
public:
    GeneratedResults();
    GeneratedResults(std::vector <cv::Mat> result);
    void setGeneratedResults(std::vector <cv::Mat> result);
    void save();
};

#endif // GENERATEDRESULTS_H
