#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types_c.h>

#include <iostream>
#include "debug.h"

#include "puzzlecutter.h"

std::vector <std::string> files {
    //std::string("D:\\Dmitriy\\Luxoft\\puzzles\\cat_with_rotations.jpg"),
    //std::string("D:\\Dmitriy\\Luxoft\\puzzles\\puzzle_rotation.jpg"),
    std::string("D:\\Dmitriy\\Luxoft\\puzzles\\new_cat.jpg")
    //std::string("D:\\Dmitriy\\Luxoft\\puzzles\\for_test.jpg")
    //std::string("D:\\Dmitriy\\Luxoft\\puzzles\\photo.jpg")
    //std::string("D:\\Dmitriy\\Luxoft\\puzzles\\test\\test_scanned_small.png")
    //::string("D:\\Dmitriy\\Luxoft\\puzzles\\test\\test.png")
    //std::string("D:\\Dmitriy\\Luxoft\\puzzles\\test\\rotate_test.png")
};


#define REVPOINT(p) cv::Point(p.y, p.x)

void merge_puzzles(const Puzzle &p1, const Puzzle &p2, cv::Mat &m)
{
    PRINT(p1.getPuzzleId(), p2.getPuzzleId());
    const cv::Mat &p1_img = p1.getImage(Puzzle::PuzzleAlignment::BottomSide);
    const cv::Mat &p2_img = p2.getImage(Puzzle::PuzzleAlignment::TopSide);
    m = cv::Mat(std::max(p1.getHeight(), p2.getHeight()) * 2, std::max(p1.getWidth(), p2.getWidth())*2,
                   p1_img.type(), cv::Scalar(0, 0, 0));
    int dx = (m.cols - p1_img.cols) / 2;
    for(int i = 0; i < p1_img.rows; i++)
    {
        for(int j = 0; j < p1_img.cols; j++)
        {
            m.at <cv::Vec3b> (i, j + dx) = p1_img.at <cv::Vec3b> ( i, j);
        }
    }
    cv::Point bottom_left = REVPOINT(p1.getCorner(Puzzle::PuzzleAlignment::BottomSide, 0)) + cv::Point(0, dx) -
            REVPOINT(p2.getCorner(Puzzle::PuzzleAlignment::TopSide, 2));
    PRINT(bottom_left);


    for(int i = 0; i < p2_img.rows; i++)
    {
        for(int j = 0; j < p2_img.cols; j++)
        {
            if(p2_img.at <cv::Vec3b> (i, j) != cv::Vec3b(0,0,0))
            m.at <cv::Vec3b> (i + bottom_left.x, j + bottom_left.y) = p2_img.at <cv::Vec3b> (i, j);
        }
    }
}

int main()
{
    PuzzleCutter::setBackgroundThreshold(13);
    PuzzleCutter::setFeatureWindowSize(4);
    PuzzleCutter::setErosionSize(1);
    std::cout << "Processing...\n";
    std::unique_ptr <std::vector <Puzzle>> puzzleContainer = PuzzleCutter::getInstanse().cutImage(files);
    std::cout << "Done!\n";
    int a, b;
    while(1)
    {
        std::cin >> a >> b;
        cv::Mat m;
        merge_puzzles(puzzleContainer->at(a * 4), puzzleContainer->at(b * 4), m);
        cv::imshow("winn", m);
        cv::waitKey();
    }
    //for(Puzzle &p : *puzzleContainer)
//    while(1)
//    {
//        std::cin >> a;
//        Puzzle &p = puzzleContainer->at(a * 4);
//        cv::imshow("alignbottom", p.getImage(Puzzle::PuzzleAlignment::BottomSide));
//        cv::imshow("alignleft", p.getImage(Puzzle::PuzzleAlignment::LeftSide));
//        cv::imshow("aligntop", p.getImage(Puzzle::PuzzleAlignment::TopSide));
//        cv::imshow("alignright", p.getImage(Puzzle::PuzzleAlignment::RightSide));
//        cv::waitKey();
//    }
    return 0;
}

