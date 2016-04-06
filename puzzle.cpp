#include "puzzle.h"

#include <chrono>
#include <thread>

Puzzle::Puzzle(int puzzle_id, cv::Mat img,  std::vector<cv::Point> corners)
    : mPuzzleId(puzzle_id)
{
    if(corners[0].x > corners[1].x)
        std::swap(corners[0], corners[1]);
    if(corners[2].x > corners[3].x)
        std::swap(corners[2], corners[3]);


    // bottom side
    double angle = geometry::vector_angle(geometry::toVector(corners[0], corners[1]));
    angle = angle/geometry::pi * 180;
    if(angle >= 180.0)
        angle -= 180.0;
    if(angle >= 90.0)
        angle = angle-180.0;
    cv::Mat img_rotate;
    PuzzleCutter::getInstanse().rotateImg(img, angle, img_rotate);
    mCorners[0] = corners;
    cv::Point center(img.cols / 2, img.rows / 2);
    for(cv::Point &p : mCorners[0])
        p = geometry::vector_rotate(p - center, -angle/180*geometry::pi) + center;
    processPuzzle(img_rotate, Puzzle::PuzzleAlignment::BottomSide);


    // left side
    angle = geometry::vector_angle(geometry::toVector(corners[0], corners[2])) - geometry::pi/2;
    angle = angle/geometry::pi * 180;
    PRINT(angle + 0);
    if(angle >= 180.0)
        angle -= 180.0;
    if(angle >= 90.0)
        angle = angle-180.0;
    PuzzleCutter::getInstanse().rotateImg(img, angle, img_rotate);
    mCorners[1] = corners;
    for(cv::Point &p : mCorners[1])
        p = geometry::vector_rotate(p - center, -angle/180*geometry::pi) + center;
    processPuzzle(img_rotate, Puzzle::PuzzleAlignment::LeftSide);


    // top side
    angle = geometry::vector_angle(geometry::toVector(corners[2], corners[3]));
    angle = angle/geometry::pi * 180;
    if(angle >= 180.0)
        angle -= 180.0;
    if(angle >= 90.0)
        angle = angle-180.0;
    PuzzleCutter::getInstanse().rotateImg(img, angle, img_rotate);
    mCorners[2] = corners;
    for(cv::Point &p : mCorners[2])
        p = geometry::vector_rotate(p - center, -angle/180*geometry::pi) + center;
    processPuzzle(img_rotate, Puzzle::PuzzleAlignment::TopSide);


    // right side
    angle = geometry::vector_angle(geometry::toVector(corners[1], corners[3])) - geometry::pi / 2;
    angle = angle/geometry::pi * 180;
    if(angle >= 180.0)
        angle -= 180.0;
    if(angle >= 90.0)
        angle = angle-180.0;
    PuzzleCutter::getInstanse().rotateImg(img, angle, img_rotate);
    mCorners[3] = corners;
    for(cv::Point &p : mCorners[3])
        p = geometry::vector_rotate(p - center, -angle/180*geometry::pi) + center;
    processPuzzle(img_rotate, Puzzle::PuzzleAlignment::RightSide);

}


int Puzzle::getPuzzleId() const
{
    return mPuzzleId;
}


const cv::Mat &Puzzle::getImage(PuzzleAlignment align) const
{
    return mPuzzleImg[size_t(align)];
}

int Puzzle::getHeight() const
{
    return mPuzzleImg[0].rows;
}

int Puzzle::getWidth() const
{
    return mPuzzleImg[0].cols;
}

cv::Point Puzzle::getCorner(Puzzle::PuzzleAlignment align, int corner) const
{
    return mCorners[size_t(align)][corner];
}


void Puzzle::processPuzzle(const cv::Mat &img, PuzzleAlignment align)
{
    auto toIdx = [&img](int i, int j){return i * img.cols + j;};

    static std::vector <cv::Point> moves {
                    cv::Point(0, -1),
                    cv::Point(0, 1),
                    cv::Point(-1, 0),
                    cv::Point(1, 0),
                    cv::Point(1, -1),
                    cv::Point(1, 1),
                    cv::Point(-1, -1),
                    cv::Point(-1, 1)};

    std::vector <int> matr(img.cols * img.rows, -1);

    // split image to puzzle and background
    std::queue <cv::Point> q;
    q.push(cv::Point(0, 0));
    while(!q.empty())
    {
        cv::Point cur = q.front();
        q.pop();
        for(cv::Point &dt : moves)
        {
            cv::Point p = cur + dt;
            if(p.x >= 0 && p.x < img.rows &&
                p.y >= 0 && p.y < img.cols
                    && matr[toIdx(p.x, p.y)] == -1)
            {
                cv::Vec3b clr = img.at<cv::Vec3b>(p.x, p.y);
                if(clr == cv::Vec3b(0,0,0))
                {
                    matr[toIdx(p.x, p.y)] = 0;
                    q.push(p);
                }
            }
        }
    }

    // cut background border around puzzle
    cv::Point min_pt(img.rows-1, img.cols-1);
    cv::Point max_pt(0, 0);
    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            if(matr[toIdx(i, j)] == -1)
            {
                if(i < min_pt.x)
                    min_pt.x = i;
                if(i > max_pt.x)
                    max_pt.x = i;
                if(j < min_pt.y)
                    min_pt.y = j;
                if(j > max_pt.y)
                    max_pt.y = j;

                matr[toIdx(i, j)] = 1;
                bool flag = false;
                for(cv::Point &dt : moves)
                {
                    cv::Point p = cv::Point(i, j) + dt;
                    if(p.x >= 0 && p.x < img.rows &&
                        p.y >= 0 && p.y < img.cols
                            && matr[toIdx(p.x, p.y)] == 0)
                    {
                        flag = true;
                    }
                }
                if(flag)
                    matr[toIdx(i, j)] = 2;
            }
        }
    }


    mPuzzleImg[size_t(align)] = cv::Mat(max_pt.x - min_pt.x + 1,  max_pt.y - min_pt.y, img.type(), cv::Scalar(0,0,0));
    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            if(matr[toIdx(i, j)] > 0)
            {
                mPuzzleImg[size_t(align)].at<cv::Vec3b>(i-min_pt.x, j - min_pt.y) = img.at <cv::Vec3b>(i, j);
            }
        }
    }


    // correct corner points positions and find top and bottom edge points
//    std::vector <cv::Point> edges;
//    std::function<void(cv::Point)>  fillEdgesVector =
//            [&matr, &edges, &moves, &toIdx, &min_pt, &img, &fillEdgesVector](cv::Point pt)
//    {
//        edges.push_back(pt - min_pt);
//        std::swap(edges.back().x, edges.back().y);
//        matr[toIdx(pt.x, pt.y)] = 3;
//        for(cv::Point dt : moves)
//        {
//            cv::Point p = pt + dt;
//            if(p.x >= 0 && p.x < img.rows &&
//                p.y >= 0 && p.y < img.cols
//                    && matr[toIdx(p.x, p.y)] > 0 && matr[toIdx(p.x, p.y)] == 2)
//            {
//                if(std::abs(edges.back().x+min_pt.y - p.y) + std::abs(edges.back().y+min_pt.x - p.x) < 20)
//                    fillEdgesVector(p);
//                else
//                    break;
//            }
//        }
//    };
    correctCorners(matr, img, align);
//    fillEdgesVector(cv::Point(mCorners[0].y, mCorners[0].x));
    for(cv::Point &p : mCorners[size_t(align)])
    {
        p -= cv::Point(min_pt.y, min_pt.x);
        cv::circle(mPuzzleImg[size_t(align)], p, 3, cv::Scalar(0, 255, 255), 1);
    }
//    std::vector <std::pair <int, int> > cornerIdx(4, {0, std::numeric_limits<int>::max()});
//    for(int i = 0; i < int(edges.size()); i++)
//    {
//        for(int j = 0; j < 4; j++)
//        {
//            int v = std::abs(mCorners[j].x - edges[i].x) + std::abs(mCorners[j].y - edges[i].y);
//            if(v < cornerIdx[j].second)
//                cornerIdx[j] = {i, v};
//        }
//    }
//    if(cornerIdx[1].first < cornerIdx[2].first)
//    {
//        for(int i = 0; i <= cornerIdx[1].first; i++)
//            this->mEdgeBottomPoints.push_back(edges[i]);
//        for(int i = cornerIdx[2].first; i >= cornerIdx[3].first; i--)
//            this->mEdgeTopPoints.push_back(edges[i]);
//    }
//    else
//    {
//        for(int i = int(edges.size())-1; i >= cornerIdx[1].first; i--)
//            this->mEdgeBottomPoints.push_back(edges[i]);
//        for(int i = cornerIdx[2].first; i <= cornerIdx[3].first; i++)
//            this->mEdgeTopPoints.push_back(edges[i]);
//    }
}

void Puzzle::correctCorners(const std::vector<int> &matr, const cv::Mat &img, PuzzleAlignment align)
{
    static int win_size = 5;
    static int win_value_size = 3;
    auto toIdx = [&img](int i, int j){return i * img.cols + j;};

    for(cv::Point &corner : mCorners[size_t(align)])
    {
        cv::Point best_pt;
        int best_value = 0;
        for(int i = corner.x-win_size; i <= corner.x + win_size; i++)
        {
            for(int j = corner.y-win_size; j <= corner.y + win_size; j++)
            {
                if(i >= 0 && i < img.cols &&
                    j >= 0 && j < img.rows
                        && matr[toIdx(j, i)] == 2)
                {
                    int value = 0;
                    for(int x = i - win_value_size; x <= i + win_value_size; x++)
                    {
                        for(int y = j - win_value_size; y <= j + win_value_size; y++)
                        {
                            if(x >= 0 && x < img.cols &&
                                y >= 0 && y < img.rows)
                                value += (matr[toIdx(y, x)] == 0);
                            else
                                value++;
                        }
                    }
                    if(value > best_value)
                    {
                        best_value = value;
                        best_pt = cv::Point(i, j);
                    }
                }
            }
        }
        corner = best_pt;
    }
}


