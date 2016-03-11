#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef VIEW_H
#define VIEW_H


class View
{
public:
    View();
    void showGeneratedPuzzles(cv::Mat *generatedPuzzles);
private:
    void showMask(cv::Mat mask);
};

#endif // VIEW_H
