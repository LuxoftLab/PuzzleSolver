#include "generatedresults.h"

const char* generatedPuzzlesParts = "C://Users//KirillUser//Documents//CannyStill1//airplane_part_";

GeneratedResults::GeneratedResults(std::vector <cv::Mat> result)
{
    for(auto it : result)
    {
        this->generatedPuzzles.push_back(it);
    }
}

GeneratedResults::GeneratedResults()
{

}

void GeneratedResults::setGeneratedResults(std::vector<cv::Mat> result)
{
    for(auto it : result)
    {
        this->generatedPuzzles.push_back(it);
    }
}

void GeneratedResults::save()
{
    int cnt = 1;
    for(auto it : this->generatedPuzzles)
    {
        std::string path = generatedPuzzlesParts;
        cv::namedWindow("imgShuffled ", CV_WINDOW_AUTOSIZE);
        cv::imshow("imgShuffled ", it);
        path += char(cnt + 48);
        cnt++;
        path += ".jpg";
        cv::imwrite(path, it);
        cv::waitKey(0);
    }
}

