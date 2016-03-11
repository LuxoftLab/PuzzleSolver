#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#ifndef GENERATEDRESULTS_H
#define GENERATEDRESULTS_H


class GeneratedResults
{
public:
    GeneratedResults();
    void save(cv::Mat generatedPuzzles);
};

#endif // GENERATEDRESULTS_H
