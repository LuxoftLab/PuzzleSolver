#include "puzzle.h"

#include <chrono>
#include <thread>

std::vector <cv::Point> Puzzle::sMoves = {
    cv::Point(0, -1),
    cv::Point(0, 1),
    cv::Point(-1, 0),
    cv::Point(1, 0),
    cv::Point(1, -1),
    cv::Point(1, 1),
    cv::Point(-1, -1),
    cv::Point(-1, 1)};

Puzzle::Puzzle(int puzzle_id, cv::Mat img,  std::vector<cv::Point> corners)
    : mPuzzleId(puzzle_id)
{
    std::vector <int> matr(img.cols * img.rows, -1);
    splitBackground(matr, img);
    findPuzzle(matr, img);
    correctCorners(matr, img, corners);


    // bottom side
    double angle = geometry::vector_angle(geometry::toVector(corners[0], corners[1]));
    angle = geometry::angle_fix(angle/geometry::pi * 180);
    cv::Mat img_rotate;
    PuzzleCutter::getInstanse().rotateImg(img, angle, img_rotate);
    mCorners[0] = corners;
    cv::Point center(img.cols / 2, img.rows / 2);
    for(cv::Point &p : mCorners[0])
        p = geometry::vector_rotate(p - center, -angle/180*geometry::pi) + center;
    processPuzzle(img_rotate, Puzzle::PuzzleAlignment::BottomSide);


    // left side
    angle = geometry::vector_angle(geometry::toVector(corners[0], corners[2])) - geometry::pi/2;
    angle = geometry::angle_fix(angle/geometry::pi * 180);
    PuzzleCutter::getInstanse().rotateImg(img, angle, img_rotate);
    mCorners[1] = corners;
    for(cv::Point &p : mCorners[1])
        p = geometry::vector_rotate(p - center, -angle/180*geometry::pi) + center;
    processPuzzle(img_rotate, Puzzle::PuzzleAlignment::LeftSide);


    // top side
    angle = geometry::vector_angle(geometry::toVector(corners[2], corners[3]));
    angle = geometry::angle_fix(angle/geometry::pi * 180);
    PuzzleCutter::getInstanse().rotateImg(img, angle, img_rotate);
    mCorners[2] = corners;
    for(cv::Point &p : mCorners[2])
        p = geometry::vector_rotate(p - center, -angle/180*geometry::pi) + center;
    processPuzzle(img_rotate, Puzzle::PuzzleAlignment::TopSide);


    // right side
    angle = geometry::vector_angle(geometry::toVector(corners[1], corners[3])) - geometry::pi/2;
    angle = geometry::angle_fix(angle/geometry::pi * 180);
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

const std::vector<cv::Point> &Puzzle::getEdgePoints(Puzzle::PuzzleAlignment align) const
{
    return mEdgePoints[size_t(align)];
}

const std::vector<cv::Scalar> &Puzzle::getEdgeColos(Puzzle::PuzzleAlignment align) const
{
    return mEdgeColors[size_t(align)];
}

bool Puzzle::isStraightSide(Puzzle::PuzzleAlignment align) const
{
    static int max_dist = 5;

    switch(align)
    {
    case PuzzleAlignment::BottomSide:
        return mEdgeRect[size_t(align)].second.y - mEdgeRect[size_t(align)].first.y < max_dist;
        break;
    case PuzzleAlignment::LeftSide:
        return mEdgeRect[size_t(align)].second.x - mEdgeRect[size_t(align)].first.x < max_dist;
        break;
    case PuzzleAlignment::TopSide:
        return mEdgeRect[size_t(align)].second.y - mEdgeRect[size_t(align)].first.y < max_dist;
        break;
    case PuzzleAlignment::RightSide:
        return mEdgeRect[size_t(align)].second.x - mEdgeRect[size_t(align)].first.x < max_dist;
        break;
    default:
        break;
    }
    return false;
}


void Puzzle::processPuzzle(const cv::Mat &img, PuzzleAlignment align)
{
    auto toIdx = [&img](int i, int j){return i * img.cols + j;};

    std::vector <int> matr(img.cols * img.rows, -1);

    splitBackground(matr, img);

    auto border_pts = findPuzzle(matr, img);
    cv::Point &min_pt = border_pts.first;
    cv::Point &max_pt = border_pts.second;

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


    std::vector <cv::Point> edges;
    std::function<void(cv::Point)>  fillEdgesVector =
            [&matr, &edges, &sMoves, &toIdx, &min_pt, &img, &fillEdgesVector](cv::Point pt)
    {
        edges.push_back(pt - min_pt);
        std::swap(edges.back().x, edges.back().y);
        matr[toIdx(pt.x, pt.y)] = 3;
        for(cv::Point dt : sMoves)
        {
            cv::Point p = pt + dt;
            if(p.x >= 0 && p.x < img.rows &&
                p.y >= 0 && p.y < img.cols
                    && matr[toIdx(p.x, p.y)] == 2)
            {
                if(std::abs(edges.back().x+min_pt.y - p.y) + std::abs(edges.back().y+min_pt.x - p.x) < 10)
                    fillEdgesVector(p);
                else
                    break;
            }
        }
    };

    correctCorners(matr, img, mCorners[size_t(align)]);
    fillEdgesVector(cv::Point(mCorners[size_t(align)][0].y, mCorners[size_t(align)][0].x));
    for(cv::Point &p : mCorners[size_t(align)])
    {
        p -= cv::Point(min_pt.y, min_pt.x);
        //cv::circle(mPuzzleImg[size_t(align)], p, 3, cv::Scalar(0, 255, 255), 1);
    }
//    cv::circle(mPuzzleImg[size_t(align)], mCorners[size_t(align)][0], 3, cv::Scalar(0, 255, 255), 1);
//    cv::circle(mPuzzleImg[size_t(align)], mCorners[size_t(align)][1], 3, cv::Scalar(0, 0, 255), 1);
//    cv::circle(mPuzzleImg[size_t(align)], mCorners[size_t(align)][2], 3, cv::Scalar(255, 112, 112), 1);
    std::vector <std::pair <int, int> > cornerIdx(4, {0, std::numeric_limits<int>::max()});

    for(int i = 0; i < int(edges.size()); i++)
    {
        for(int j = 0; j < 4; j++)
        {
            int v = std::abs(mCorners[size_t(align)][j].x - edges[i].x) +
                    std::abs(mCorners[size_t(align)][j].y - edges[i].y);
            if(v < cornerIdx[j].second)
                cornerIdx[j] = {i, v};
        }
    }

    if(align == Puzzle::PuzzleAlignment::BottomSide)
    {
        if(cornerIdx[1].first < cornerIdx[2].first)
            for(int i = 0; i <= cornerIdx[1].first; i++)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
        else
            for(int i = int(edges.size())-1; i >= cornerIdx[1].first; i--)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
    }
    else if(align == Puzzle::PuzzleAlignment::LeftSide)
    {
        if(cornerIdx[1].first < cornerIdx[2].first)
            for(int i = cornerIdx[2].first; i < int(edges.size()); i++)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
        else
            for(int i = cornerIdx[2].first; i >= 0; i--)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
    }
    else if(align == Puzzle::PuzzleAlignment::TopSide)
    {
        if(cornerIdx[1].first < cornerIdx[2].first)
            for(int i = cornerIdx[2].first; i >= cornerIdx[3].first; i--)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
        else
            for(int i = cornerIdx[2].first; i <= cornerIdx[3].first; i++)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
    }
    else if(align == Puzzle::PuzzleAlignment::RightSide)
    {
        if(cornerIdx[1].first < cornerIdx[2].first)
            for(int i = cornerIdx[3].first; i >= cornerIdx[1].first; i--)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
        else
            for(int i = cornerIdx[3].first; i <= cornerIdx[1].first; i++)
                this->mEdgePoints[size_t(align)].push_back(edges[i]);
    }

    mEdgeRect[size_t(align)].first.x = std::numeric_limits <int>::max();
    mEdgeRect[size_t(align)].first.y = std::numeric_limits <int>::max();
    mEdgeRect[size_t(align)].second.x = std::numeric_limits <int>::min();
    mEdgeRect[size_t(align)].second.y = std::numeric_limits <int>::min();

    for(cv::Point &p : mEdgePoints[size_t(align)])
    {
        mEdgeColors[size_t(align)].push_back(meanColor(align, p));
        mEdgeRect[size_t(align)].first.x = std::min(mEdgeRect[size_t(align)].first.x, p.x);
        mEdgeRect[size_t(align)].first.y = std::min(mEdgeRect[size_t(align)].first.y, p.y);
        mEdgeRect[size_t(align)].second.x = std::max(mEdgeRect[size_t(align)].second.x, p.x);
        mEdgeRect[size_t(align)].second.y = std::max(mEdgeRect[size_t(align)].second.y, p.y);
    }
}

void Puzzle::splitBackground(std::vector<int> &matr, const cv::Mat &img)
{
    auto toIdx = [&img](int i, int j){return i * img.cols + j;};

    std::queue <cv::Point> q;
    q.push(cv::Point(0, 0));
    while(!q.empty())
    {
        cv::Point cur = q.front();
        q.pop();
        for(cv::Point &dt : sMoves)
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
}

std::pair<cv::Point, cv::Point> Puzzle::findPuzzle(std::vector<int> &matr, const cv::Mat &img)
{
    auto toIdx = [&img](int i, int j){return i * img.cols + j;};

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
                for(cv::Point &dt : sMoves)
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

    return std::make_pair(min_pt, max_pt);
}

void Puzzle::correctCorners(const std::vector<int> &matr, const cv::Mat &img, std::vector<cv::Point> &corners)
{
    static int win_size = 5;
    static int win_value_size = 3;
    auto toIdx = [&img](int i, int j){return i * img.cols + j;};

    for(cv::Point &corner : corners)
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

cv::Scalar Puzzle::meanColor(Puzzle::PuzzleAlignment align, cv::Point p)
{
    static int win_size = 7;

    int color[3] = {0, 0, 0};
    int total = 0;
    for(int i = p.x - win_size; i <= p.x + win_size; i++)
    {
        for(int j = p.y - win_size; j <= p.y + win_size; j++)
        {
            if(i >= 0 && j < mPuzzleImg[size_t(align)].rows &&
               j >= 0 && i < mPuzzleImg[size_t(align)].cols
                    && mPuzzleImg[size_t(align)].at <cv::Vec3b> (j, i) != cv::Vec3b::all(0))
            {
                for(int k = 0; k < 3; k++)
                    color[k] += mPuzzleImg[size_t(align)].at <cv::Vec3b> (j, i)[k];
                total++;
            }

        }
    }
    if(total == 0)
        return cv::Scalar(0, 0, 0);
    else
        return cv::Scalar(color[0] / total, color[1] / total, color[2] / total);
}


