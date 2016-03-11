#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>
#include <iostream>
#include <string>
#include "generator.h"

PuzzleGenerator::PuzzleGenerator()
{

}


void PuzzleGenerator::rotatePuzzle(const cv::Mat & src, cv::Mat & dst)
{
    double degrees = rand() % 360;
    cv::Point2f center(src.cols/2.0, src.rows/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, degrees, 1.0);
    cv::Rect bbox = cv::RotatedRect(center,src.size(), degrees).boundingRect();
    cv::Scalar white(255, 255, 255);

    rot.at<double>(0,2) += bbox.width/2.0 - center.x;
    rot.at<double>(1,2) += bbox.height/2.0 - center.y;

    cv::warpAffine(src, dst, rot, bbox.size(),
                   cv::INTER_LINEAR,cv::BORDER_CONSTANT, white);
}

void PuzzleGenerator::drawPuzzle(cv::Mat mask,
                int koefWidth,
                int koefHeight,
                int i, int j,
                int sign, int sign2,
                int angle)
{
    cv::ellipse( mask, cv::Point( j - (koefWidth * sign), i - (koefHeight * sign2)),
       cv::Size( koefWidth, koefHeight ), angle, 0, 360, clrBlack, KOEF_SET_MASK, 8 );

    cv::line(mask, cv::Point( j - ( sign ? (koefWidth * sign) / 2 :  (koefWidth)),
                              i - ( sign ?  koefHeight : (sign2 * koefHeight / 2))),
                   cv::Point( j + ( sign ? (-sign * koefWidth / 2) : (koefWidth)),
                              i - ( sign ? (-koefHeight ) : (sign2 * koefHeight / 2))),
                   clrWhite, KOEF_SET_MASK + 4);

    cv::ellipse( mask, cv::Point( j - (sign ? (koefWidth * sign) / 2 : ( koefWidth )),
                                  i - (sign ?  koefHeight : sign2 * koefHeight / 2 )),
         cv::Size( 5.0, 5.0 ), (angle + 90) % 180, 0, 360, clrBlack, KOEF_SET_MASK, 8 );


    cv::line(mask, cv::Point( sign ? j - (koefWidth * sign) : j - 3 * koefWidth / 2,
                              sign ? i - 3 * koefHeight / 2 : i - sign2 * 3 * koefHeight / 2),
                   cv::Point( sign ? j - (3 * sign) : j - 3 * koefWidth / 2 + 3,
                              sign ? i - 3 * koefHeight / 2 + 3 : i - sign2 * 3), clrWhite, 8);

    cv::line(mask, cv::Point( sign ? j : j - 3 * koefWidth / 2,
                              sign ? i - 3 * koefHeight / 2 : i - sign2 * 3 * koefHeight / 2),
                   cv::Point( sign ? j : j - 3 * koefWidth / 2 + 3,
                              sign ? i : i - sign2 * 3),
                              sign ? clrBlack : clrWhite,
                              sign ? KOEF_SET_MASK : 8);

    cv::ellipse( mask, cv::Point( sign ? j - (koefWidth * sign) / 2 : j + koefWidth,
                                  sign ? i + koefHeight : i - sign2 * koefHeight / 2 ),
                 cv::Size( 5.0, 5.0 ), sign ? (angle + 90) % 180 : 90, 0, 360,
                 clrBlack, KOEF_SET_MASK, 8 );

    if(sign2)
    {
        cv::line(mask, cv::Point( j - 3 * koefWidth / 2, i),
                       cv::Point(j, i), clrBlack, KOEF_SET_MASK);
    }

    cv::line(mask, cv::Point( sign ? j - (koefWidth * sign) : j + 3 * koefWidth / 2,
                              sign ? i + 3 * koefHeight / 2 : i - sign2 * 3 * koefHeight / 2),
                   cv::Point( sign ? j - (3 * sign) : j + 3 * koefWidth / 2 - 3,
                              sign ? i + 3 * koefHeight / 2 - 3 : i - sign2 * 3), clrWhite, 8);

    cv::line(mask, cv::Point( j, i),
                   cv::Point( sign ? j : j + 3 * koefWidth / 2,
                              sign ? i + 3 * koefHeight / 2 : i),
                   clrBlack, KOEF_SET_MASK);

    cv::line(mask, cv::Point( sign ? j + sign : j - 6,
                              sign ? i - 6 : i + sign2 * 1 ),
                   cv::Point( sign ? j + sign : j + 6,
                              sign ? i + 6 : i + sign2 * 1), clrWhite, KOEF_SET_MASK + 1);
}

void PuzzleGenerator::setPuzzleForm(cv::Mat mask)
{
    int koefWidth = 11;
    int koefHeight = 10;

    for(int i = sz.height / 2; i < IMG_SZ_HEIGHT; i += sz.height)
    {
        for(int j = sz.width; j < IMG_SZ_WIDTH; j += sz.width)
        {
            if(rand() % 2)
            {
                drawPuzzle(mask, koefWidth, koefHeight, i, j, 1, 0, 90);
            }
            else
            {
                drawPuzzle(mask, koefWidth, koefHeight, i, j, -1, 0, 90);
            }
        }
    }

    for(int i = sz.height; i < IMG_SZ_HEIGHT; i += sz.height)
    {
        for(int j = sz.width / 2; j < IMG_SZ_WIDTH; j += sz.width)
        {
            if(rand() % 2)
            {
                drawPuzzle(mask, koefWidth, koefHeight, i, j, 0, 1, 0);
            }
            else
            {
                drawPuzzle(mask, koefWidth, koefHeight, i, j, 0, -1, 90);
            }
        }
    }
}

void PuzzleGenerator::setMask(cv::Mat imgOriginal)
{
    cv::Mat mask;
    cv::Size maskSize(IMG_SZ_WIDTH, IMG_SZ_HEIGHT);
    cv::Point2i ptRowBegin(0,0);
    cv::Point2i ptRowEnd(0,0);
    cv::resize(imgOriginal, mask, maskSize);
    cv::Scalar clrBlack(255, 255, 255);
    cv::Scalar clrWhite(0, 0, 0);
    mask.setTo(clrBlack);

    for(int i = 0; i <= IMG_SZ_HEIGHT; i += sz.height)
    {
        cv::Point2i ptColBegin(0,i);
        cv::Point2i ptColEnd(IMG_SZ_WIDTH,i);
        cv::line(mask, ptColBegin, ptColEnd, clrWhite, KOEF_SET_MASK);
    }

    for(int i = 0; i <= IMG_SZ_WIDTH; i += sz.width)
    {
        cv::Point2i ptColBegin(i, 0);
        cv::Point2i ptColEnd(i, IMG_SZ_HEIGHT);
        cv::line(mask, ptColBegin, ptColEnd, clrWhite, KOEF_SET_MASK);
    }

    setPuzzleForm(mask);

    cv::namedWindow("mask", CV_WINDOW_AUTOSIZE);
    cv::imshow("mask", mask);

    cv::imwrite(maskPath, mask);

    cv::waitKey(0);

}


void PuzzleGenerator::makeSimplePuzzle(cv::Mat imgOriginal)
{
    cv::Mat mask = cv::imread(maskPath);
    cv::Mat result;
    cv::Size resultSize(IMG_SZ_WIDTH, IMG_SZ_HEIGHT);
    cv::resize(imgOriginal, imgOriginal, resultSize);

    //cv::addWeighted(imgOriginal, 0.3, mask, 0.7, 0, result);

    cv::bitwise_and(imgOriginal, mask, result);
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
    cv::imshow("result", result);

    cv::imwrite(resultPath, result);

    cv::waitKey(0);
}



void PuzzleGenerator::floodFillPuzzle(cv::Mat &puzzle, cv::Point pxl, int cnt, cv::Point center)
{
    used[pxl.y][pxl.x] = 1;

    cv::Vec3b color;
    cv::Vec3b basicColor(5, 5, 5);
    cv::Point to;

    if((pxl.x + 1) < center.x + FLOOD_CONST_X && (used[pxl.y][pxl.x + 1] == 0))
    {
        to = cv::Point(pxl.x + 1, pxl.y);
        color = puzzle.at<cv::Vec3b>(to);
        if(color[0] > basicColor[0] &&
           color[1] > basicColor[1] &&
           color[2] > basicColor[2])
        {
            floodFillPuzzle(puzzle, to, cnt, center);
        }

    }

    if((pxl.y + 1) < center.y + FLOOD_CONST_Y && (used[pxl.y + 1][pxl.x] == 0))
    {
        to = cv::Point(pxl.x, pxl.y + 1);
        color = puzzle.at<cv::Vec3b>(to);
        if(color[0] > basicColor[0] &&
           color[1] > basicColor[1] &&
           color[2] > basicColor[2])
        {
            floodFillPuzzle(puzzle, to, cnt, center);
        }

    }

    if(pxl.y - 1 > 0 && (used[pxl.y - 1][pxl.x] == 0))
    {
        to = cv::Point(pxl.x, pxl.y - 1);
        color = puzzle.at<cv::Vec3b>(to);
        if(color[0] > basicColor[0] &&
           color[1] > basicColor[1] &&
           color[2] > basicColor[2])
        {
            floodFillPuzzle(puzzle, to, cnt, center);
        }
    }

    if(pxl.x - 1 > 0 && (used[pxl.y][pxl.x - 1] == 0))
    {
        to = cv::Point(pxl.x - 1, pxl.y);
        color = puzzle.at<cv::Vec3b>(to);
        if(color[0] > basicColor[0] &&
           color[1] > basicColor[1] &&
           color[2] > basicColor[2])
        {
            floodFillPuzzle(puzzle, to, cnt, center);
        }
    }
}

int PuzzleGenerator::shufflePuzzles()
{
    cv::Mat imgOriginal;		// input image

    std::vector<cv::Mat> imageParts;

    imgOriginal = cv::imread(resultPath);

    if (imgOriginal.empty()) {									// if unable to open image
        std::cout << "error: image not read from file\n\n";		// show error message on command line
        return(0);												// and exit program
    }

    cv::Size genSZ(5*(IMG_SZ_WIDTH  + (AMOUNT+1)) / 2,
                   6*(IMG_SZ_HEIGHT + (AMOUNT+1)) / 2);
    cv::Size rotateSZ(5 * IMG_SZ_WIDTH  / (AMOUNT * 2),
                      6 * IMG_SZ_HEIGHT / (AMOUNT * 2));
    cv::Mat imgGenerated;
    cv::resize(imgOriginal, imgGenerated, genSZ, 0, 0, cv::INTER_CUBIC);

    imgGenerated.setTo(cv::Scalar(0, 0, 0));
    imgGenerated.setTo(cv::Scalar(255, 255, 255));

    int cnt = 0;
    cv::Mat temp;
    for(int i = 0; i < IMG_SZ_HEIGHT; i += sz.height)
    {
        for(int j = 0; j < IMG_SZ_WIDTH; j += sz.width)
        {
            cv::Rect myRoi;
            int midXROI;
            int midYROI;
            if(j + sz.width  + KOEF_SHUFFLE < IMG_SZ_WIDTH &&
               i + sz.height + KOEF_SHUFFLE < IMG_SZ_HEIGHT)
            {
                cnt++;
                if(j - KOEF_SHUFFLE > 0 && i - KOEF_SHUFFLE > 0)
                {
                    myRoi = cv::Rect(j - KOEF_SHUFFLE, i - KOEF_SHUFFLE,
                                     sz.width  + 2*KOEF_SHUFFLE,
                                     sz.height + 2*KOEF_SHUFFLE);
                }
                else if(j - KOEF_SHUFFLE > 0)
                {
                    myRoi = cv::Rect(j - KOEF_SHUFFLE, i,
                                     sz.width  + 2*KOEF_SHUFFLE,
                                     sz.height + 2*KOEF_SHUFFLE);
                }
                else if (i - KOEF_SHUFFLE > 0)
                {
                    myRoi = cv::Rect(j, i - KOEF_SHUFFLE,
                                     sz.width  + 2*KOEF_SHUFFLE,
                                     sz.height + 2*KOEF_SHUFFLE);
                }
                else
                {
                    myRoi = cv::Rect(j, i, sz.width  + 2*KOEF_SHUFFLE,
                                           sz.height + 2*KOEF_SHUFFLE);
                }
            }
            else if(j + sz.width  + KOEF_SHUFFLE > IMG_SZ_WIDTH &&
                    i + sz.height + KOEF_SHUFFLE > IMG_SZ_HEIGHT)
            {
                cnt++;
                myRoi = cv::Rect(j - 2*KOEF_SHUFFLE, i - 2*KOEF_SHUFFLE,
                          sz.width + 2*KOEF_SHUFFLE, sz.height + 2*KOEF_SHUFFLE);
            }
            else if(j + sz.width + KOEF_SHUFFLE < IMG_SZ_WIDTH)
            {
                cnt++;
                if (j - KOEF_SHUFFLE > 0)
                {
                    myRoi = cv::Rect(j - KOEF_SHUFFLE, i - 2*KOEF_SHUFFLE,
                             sz.width + 2*KOEF_SHUFFLE, sz.height + 2*KOEF_SHUFFLE);
                }
                else
                {
                    myRoi = cv::Rect(j, i - 2*KOEF_SHUFFLE,
                                     sz.width  + 2*KOEF_SHUFFLE,
                                     sz.height + 2*KOEF_SHUFFLE);
                }
            }
            else if(i + sz.height + KOEF_SHUFFLE < IMG_SZ_HEIGHT)
            {
                cnt++;
                if (i - KOEF_SHUFFLE > 0)
                {
                    myRoi = cv::Rect(j - 2*KOEF_SHUFFLE, i - KOEF_SHUFFLE,
                              sz.width + 2*KOEF_SHUFFLE, sz.height + 2*KOEF_SHUFFLE);
                }
                else
                {
                    myRoi = cv::Rect(j - 2*KOEF_SHUFFLE, i,
                                     sz.width +  2*KOEF_SHUFFLE,
                                     sz.height + 2*KOEF_SHUFFLE);
                }
            }

            midXROI = j + ( sz.width  ) / 2;
            midYROI = i + ( sz.height ) / 2;

            cv::Point center = cv::Point(midXROI, midYROI);

            floodFillPuzzle(imgOriginal, center, cnt, center);

            cv::Mat temp2;
            temp = cv::Mat(imgGenerated, myRoi);
            cv::resize(temp, temp2, rotateSZ, 0, 0, cv::INTER_CUBIC);
            temp2.setTo(cv::Scalar(255, 255, 255));
            temp.setTo(cv::Scalar(255, 255, 255));

            for(int z = 0; z < IMG_SZ_HEIGHT; z++)
            {
                for(int x = 0; x < IMG_SZ_WIDTH; x++)
                {
                    if(used[z][x])
                    {
                        cv::Point small(x - myRoi.x, z - myRoi.y);
                        cv::Point orig(x, z);
                        temp.at<cv::Vec3b>(small) = imgOriginal.at<cv::Vec3b>(orig);
                        used[z][x] = 0;
                    }
                }
            }

            rotatePuzzle(temp, temp);
            temp.copyTo(temp2(cv::Rect(0, 0, temp.cols, temp.rows)));
            imageParts.push_back(temp2);
            // temp.~Mat();
            // temp2.~Mat();
        }
    }

    random_shuffle(imageParts.begin(), imageParts.end());
    int numW = 0;
    int numH = 0;
    for(auto it : imageParts)
    {
        it.copyTo(imgGenerated(cv::Rect(numW * it.cols, numH * it.rows, it.cols, it.rows)));
        numW++;
        if(numW == AMOUNT)
        {
            numW %= AMOUNT;
            numH++;
        }
    }

    cv::namedWindow("imgShuffled ", CV_WINDOW_AUTOSIZE);
    cv::imshow("imgShuffled ", imgGenerated);
    cv::imwrite(generatedPuzzlesPath, imgGenerated);
    cv::waitKey(0);
}



void PuzzleGenerator::cutImageOnNParts()
{
    cv::Mat imgOriginal;

    std::vector <cv::Mat> resultImages;

    cv::Mat temp;

    imgOriginal = cv::imread(generatedPuzzlesPath);

    int stepRows = imgOriginal.rows / 2;
    int stepCols = imgOriginal.cols / 2 - 20;

    for(int i = 0; i + stepRows < imgOriginal.rows; i += stepRows)
    {
        for(int j = 0; j + stepCols < imgOriginal.cols; j += stepCols)
        {
            cv::Rect myRoi;
            myRoi = cv::Rect(j, i, stepCols, stepRows);
            temp = cv::Mat(imgOriginal, myRoi);
            resultImages.push_back(temp);
        }
    }

    int cnt = 1;
    for(auto it : resultImages)
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

void PuzzleGenerator::generate(cv::Mat img)
{
    setMask(img);
    makeSimplePuzzle(img);
    shufflePuzzles();
    cutImageOnNParts();
}

///////////////////////////////////////////////////////////////////////////////
int main() {

    PuzzleGenerator generator;
    cv::Mat img = cv::imread(originalPath);
    generator.generate(img);

    return 0;
}


