#include "puzzlecutter.h"

#define REVPOINT(p) cv::Point(p.y, p.x)

int PuzzleCutter::sBackgroundThreshold = 10;
double PuzzleCutter::sRectAngleEps = geometry::pi / 15;
int PuzzleCutter::sFeatureWindowSize = 6;
int PuzzleCutter::sErosionSize = 1;

PuzzleCutter::PuzzleCutter()
    : mMoves({cv::Point(0, -1),
          cv::Point(0, 1),
          cv::Point(-1, 0),
          cv::Point(1, 0),
          /*cv::Point(1, -1),
          cv::Point(1, 1),
          cv::Point(-1, -1),
          cv::Point(-1, 1)*/})
{

}


PuzzleCutter &PuzzleCutter::getInstanse()
{
    static PuzzleCutter instanse;
    return instanse;
}

std::unique_ptr<std::vector<Puzzle> > PuzzleCutter::cutImage(const std::vector<std::string> &files)
{
    std::unique_ptr <std::vector <Puzzle> > pContainer(new std::vector <Puzzle>());

    for(const std::string &file_path : files)
    {
        mMatr.clear();
        LOG() << "Analyse file " << file_path << "\n";
        cv::Mat img = cv::imread(file_path);
#ifdef SHOW_STEP_RESULT
        cv::Mat img_copy = img.clone();
#endif
        cv::Mat dst;
        divideIntoComponents(img, dst);

#ifdef SHOW_STEP_RESULT
        cv::imshow("win", dst);
        cv::waitKey();
#endif

        // clean image from erosion
        cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                             cv::Size( 2*sErosionSize + 1, 2*sErosionSize+1 ),
                                             cv::Point( sErosionSize, sErosionSize ) );
        cv::dilate(dst, dst, element);

        mMatr.clear();
        divideIntoComponents(dst, dst);

#ifdef SHOW_STEP_RESULT
        cv::imshow("win", dst);
        cv::waitKey();
#endif

        findPuzzles(dst);
        int Id = 0;
        for(PuzzleData &puzzle : mPuzzlesData)
        {
#ifdef SHOW_STEP_RESULT
            cv::Point ctr((puzzle.maxPt.x + puzzle.minPt.x) / 2, (puzzle.maxPt.y + puzzle.minPt.y) / 2);
            cv::putText(img_copy, std::to_string(Id++).c_str(), REVPOINT(ctr), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0,0,0), 2);
#endif
//            int feature_threshold_l = sFeatureWindowSize * sFeatureWindowSize * 2;
//            int feature_threshold_r = sFeatureWindowSize * sFeatureWindowSize * 4;
//            int best_threshold = feature_threshold_r;
//            while(feature_threshold_l <= feature_threshold_r)
//            {
//                int mid = (feature_threshold_l + feature_threshold_r) >> 1;
//                findFeaturePoints(dst, puzzle, mid);
//                if(isPillarLines(dst, puzzle))
//                {
//                    best_threshold = mid;
//                    feature_threshold_l = mid+1;
//                }
//                else
//                    feature_threshold_r = mid-1;
//                puzzle.features.clear();
//            }
            int best_threshold = sFeatureWindowSize * sFeatureWindowSize * 3.2;
            findFeaturePoints(dst, puzzle, best_threshold);
            findPillarLines(dst, puzzle);


#ifdef SHOW_STEP_RESULT
            for(cv::Point p : puzzle.features)
            {
                cv::circle(img_copy, p, 4, cv::Scalar(0, 0, 255), 2);
            }
            for(std::pair <cv::Point, cv::Point> l : puzzle.pillarLines)
            {
                cv::line(img_copy, l.first, l.second, cv::Scalar(0, 0, 255), 2);
            }
            cv::imshow("win", img_copy);
            cv::waitKey();
#endif

            cv::Mat p_img(puzzle.height() * 2, puzzle.width() * 2, img.type(), cv::Scalar(0,0,0));
            auto toIdx = [&img](int i, int j){return i * img.cols + j;};
            for(int i = puzzle.minPt.x; i <= puzzle.maxPt.x; i++)
            {
                for(int j = puzzle.minPt.y; j <= puzzle.maxPt.y; j++)
                {
                    if(mMatr[toIdx(i, j)].first == puzzle.puzzleNumber)
                        p_img.at <cv::Vec3b>(i-puzzle.minPt.x + puzzle.height()/2,
                                             j-puzzle.minPt.y + puzzle.width() / 2) = img.at <cv::Vec3b>(i, j);
                    else
                        p_img.at <cv::Vec3b>(i-puzzle.minPt.x + puzzle.height()/2,
                                             j-puzzle.minPt.y + puzzle.width() / 2) = cv::Vec3b(0,0,0);
                }
            }


            // rotate puzzle and corners
            double angle = puzzleAngle(puzzle);
            PRINT(angle);
            rotateImg(p_img, angle, p_img);
            cv::Point mn(puzzle.minPt.y, puzzle.minPt.x);
            cv::Point center(p_img.cols / 2, p_img.rows / 2);
            std::vector <cv::Point> corners {puzzle.pillarLines[2].second, puzzle.pillarLines[2].first,
                                            puzzle.pillarLines[0].first, puzzle.pillarLines[0].second};
            for(cv::Point &p : corners)
            {
                p = p - mn + cv::Point(puzzle.width() / 2, puzzle.height()/2);
                p = geometry::vector_rotate(p - center, -angle / 180 * geometry::pi) + center;
            }

            for(int k = 0; k < 4; k++)
            {
                pContainer->push_back(Puzzle(puzzle.puzzleNumber, p_img, corners));

                rotateImg(p_img, 90, p_img);
                for(cv::Point &p : corners)
                {
                    //p = p - mn + pls;
                    p = geometry::vector_rotate(p - center, -geometry::pi/2) + center;
                }
                std::vector <cv::Point> tmp = corners;
                corners[0] = tmp[1];
                corners[1] = tmp[3];
                corners[2] = tmp[0];
                corners[3] = tmp[2];
            }
        }
    }



    return pContainer;
}


void PuzzleCutter::convertToEdges(const cv::Mat &src, cv::Mat &dst)
{
    static int canny_threshold = 30;
    static int canny_ratio = 3;
    static int canny_kernel_size = 3;

    cv::cvtColor(src, dst, CV_BGR2GRAY);
    cv::blur(dst, dst, cv::Size(canny_kernel_size, canny_kernel_size));
    cv::Canny(dst, dst, canny_threshold, canny_threshold * canny_ratio, canny_kernel_size);
}

void PuzzleCutter::divideIntoComponents(const cv::Mat &src, cv::Mat &dst)
{
    static cv::Vec3b background_color = cv::Vec3b(255,255,255);
    static cv::Vec3b puzzle_color = cv::Vec3b(0,0,0);
    auto toIdx = [&src](int i, int j) {return i * src.cols + j;};

    dst = src.clone();
    mMatr.resize( src.cols * src.rows, {-1, false});
    mMatr[0] = {0, false};

    // divide image to background and puzzles
    std::queue <cv::Point> q;
    q.push(cv::Point(0, 0));
    while(!q.empty())
    {
        cv::Point cur = q.front();
        q.pop();
        dst.at<cv::Vec3b>(cur.x, cur.y) = background_color;
        for(cv::Point dt: mMoves)
        {
            cv::Point p = cur + dt;
            if(p.x >= 0 && p.x < src.rows &&
                p.y >= 0 && p.y < src.cols
                    && mMatr[toIdx(p.x, p.y)].first == -1)
            {
                cv::Vec3b ground_px = src.at<cv::Vec3b>(cur.x, cur.y);
                cv::Vec3b cur_px = src.at<cv::Vec3b>(p.x, p.y);
                bool flag = true;
                for(int k = 0; k < 3; k++)
                    flag &= std::abs(ground_px[k] - cur_px[k]) < sBackgroundThreshold;
                if(flag)
                {
                    mMatr[toIdx(p.x, p.y)] = {0, false};
                    q.push(p);
                }
            }
        }
    }
    for(int i = 0; i < src.rows; i++)
        for(int j = 0; j < src.cols; j++)
            if(mMatr[toIdx(i, j)].first == -1)
                dst.at <cv::Vec3b>(i, j) = puzzle_color;

}

void PuzzleCutter::findPuzzles(const cv::Mat &src)
{
    static int min_puzzle_points = 100;

    auto toIdx = [&src](int i, int j){return i * src.cols + j;};

    int puzzle_number = 1;
    for(int i = 0; i < src.rows; i++)
    {
        for(int j = 0; j < src.cols; j++)
        {
            if(mMatr[toIdx(i, j)].first == -1)
            {
                mMatr[toIdx(i, j)] = {puzzle_number, false};
                std::queue <cv::Point> q;
                q.push(cv::Point(i, j));
                cv::Point min_pt = q.front();
                cv::Point max_pt = q.front();
                int points = 0;
                while(!q.empty())
                {
                    points++;
                    cv::Point cur = q.front();
                    q.pop();
                    if(cur.x < min_pt.x)
                        min_pt.x = cur.x;
                    if(cur.x > max_pt.x)
                        max_pt.x = cur.x;
                    if(cur.y < min_pt.y)
                        min_pt.y = cur.y;
                    if(cur.y > max_pt.y)
                        max_pt.y = cur.y;
                    for(cv::Point &dt : mMoves)
                    {
                        cv::Point p = cur + dt;
                        if(p.x >= 0 && p.x <= src.rows &&
                            p.y >= 0 && p.y <= src.cols
                                && mMatr[toIdx(p.x, p.y)].first == -1)
                        {
                            q.push(p);
                            mMatr[toIdx(p.x, p.y)] = {puzzle_number, false};
                        }
                    }
                }
                if(points >= min_puzzle_points)
                {
                    PuzzleData pdata;
                    pdata.minPt = min_pt;
                    pdata.maxPt = max_pt;
                    pdata.area = points;
                    pdata.puzzleNumber = puzzle_number;
                    mPuzzlesData.push_back(pdata);
                }
                puzzle_number++;
            }
        }
    }
}

void PuzzleCutter::findFeaturePoints(const cv::Mat &src, PuzzleData &pdata, int feature_value_threshold)
{
   // static int window_size = 4;
    //static int feature_value_threshold = window_size * window_size * 2.8;

    auto toIdx = [&src](int i, int j){return i * src.cols + j;};
    auto toFVIdx = [&src, &pdata](int i, int j)
                {return (i - pdata.minPt.x) * pdata.width() + j - pdata.minPt.y;};

    std::vector <int> feature_value(pdata.width() * pdata.height(), 0);
    for(int i = pdata.minPt.x; i <= pdata.maxPt.x; i++)
    {
        for(int j = pdata.minPt.y; j <= pdata.maxPt.y; j++)
        {
            if(mMatr[toIdx(i, j)].first == pdata.puzzleNumber)
            {
                int cnt = 0;
                std::vector <cv::Point> competitors;
                for(int x = i-sFeatureWindowSize; x <= i + sFeatureWindowSize; x++)
                {
                    for(int y = j-sFeatureWindowSize; y <= j + sFeatureWindowSize; y++)
                    {
                        if(!(x < pdata.minPt.x || x > pdata.maxPt.x
                                || y < pdata.minPt.y || y > pdata.maxPt.y))
                        {
                            if(mMatr[toIdx(i, j)].first != mMatr[toIdx(x, y)].first)
                                cnt++;
                            if(feature_value[toFVIdx(x, y)] > 0)
                                competitors.push_back(cv::Point(x, y));
                        }
                        else
                            cnt++;
                    }
                }
                if(cnt > feature_value_threshold)
                {
                    bool isMoveValue = false;
                    for(cv::Point pt : competitors)
                    {
                        if(cnt > feature_value[toFVIdx(pt.x, pt.y)])
                            feature_value[toFVIdx(pt.x, pt.y)] = 0;
                        else
                            isMoveValue = true;
                    }
                    if(!isMoveValue)
                        feature_value[toFVIdx(i, j)] = cnt++;
                }
                if(cnt > 0)
                    mMatr[toIdx(i, j)].second = true;
            }
        }
    }
    for(int i = pdata.minPt.x; i <= pdata.maxPt.x; i++)
        for(int j = pdata.minPt.y; j <= pdata.maxPt.y; j++)
            if(feature_value[toFVIdx(i, j)] > 0)
                pdata.features.push_back(cv::Point(j, i));
}

void PuzzleCutter::findPillarLines(const cv::Mat &src, PuzzleData &pdata)
{
    double cur_area = 0;
    for(int i = 0; i < int(pdata.features.size()); i++)
    {
        for(int j = i + 1; j < int(pdata.features.size()); j++)
        {
            int pt1 = -1;
            for(int k = 0; k < int(pdata.features.size()); k++)
            {
                if(k == i || k == j)
                    continue;
                double angle = geometry::vector_angle(geometry::toVector(pdata.features[i], pdata.features[j]),
                                                      geometry::toVector(pdata.features[j], pdata.features[k]));
                if(std::abs(angle - geometry::pi/2) < sRectAngleEps)
                {
                    if(pt1 == -1 || geometry::dist(pdata.features[j], pdata.features[k]) >
                            geometry::dist(pdata.features[j], pdata.features[pt1]))
                        pt1 = k;
                }
            }
            if(pt1 == -1)
                continue;
            int pt2 = -1;
            for(int k = 0; k < int(pdata.features.size()); k++)
            {
                if(k == i || k == j || k == pt1)
                    continue;
                double angle = geometry::vector_angle(geometry::toVector(pdata.features[j], pdata.features[i]),
                                                      geometry::toVector(pdata.features[i], pdata.features[k]));
                if(std::abs(angle - geometry::pi/2) < sRectAngleEps)
                {
                    if(pt2 == -1 || geometry::dist(pdata.features[i], pdata.features[k]) >
                            geometry::dist(pdata.features[i], pdata.features[pt2]))
                        pt2 = k;
                }
            }
            if(pt2 != -1)
            {
                double angle = geometry::vector_angle(geometry::toVector(pdata.features[i], pdata.features[pt2]),
                                                      geometry::toVector(pdata.features[pt2], pdata.features[pt1]));
                if(std::abs(angle - geometry::pi / 2) > sRectAngleEps)
                    continue;
                angle = geometry::vector_angle(geometry::toVector(pdata.features[j], pdata.features[pt1]),
                                               geometry::toVector(pdata.features[pt1], pdata.features[pt2]));
                if(std::abs(angle - geometry::pi / 2) > sRectAngleEps)
                    continue;
                std::vector <cv::Point> pts {pdata.features[i],
                            pdata.features[j],
                            pdata.features[pt1],
                            pdata.features[pt2]};
                double area = geometry::polygon_area(pts);
                if(area > cur_area)
                {
                    pdata.pillarLines.clear();
                    for(int k = 0; k < 4; k++)
                        pdata.pillarLines.push_back({pts[k], pts[(k + 1) % int(pts.size())]});
                    cur_area = area;
                }
            }
        }
    }
}

bool PuzzleCutter::isPillarLines(const cv::Mat &src, PuzzleCutter::PuzzleData &pdata)
{
    for(int i = 0; i < int(pdata.features.size()); i++)
    {
        for(int j = i + 1; j < int(pdata.features.size()); j++)
        {
            int pt1 = -1;
            for(int k = 0; k < int(pdata.features.size()); k++)
            {
                if(k == i || k == j)
                    continue;
                double angle = geometry::vector_angle(geometry::toVector(pdata.features[i], pdata.features[j]),
                                                      geometry::toVector(pdata.features[j], pdata.features[k]));
                if(std::abs(angle - geometry::pi/2) < sRectAngleEps)
                {
                    if(pt1 == -1 || geometry::dist(pdata.features[j], pdata.features[k]) >
                            geometry::dist(pdata.features[j], pdata.features[pt1]))
                        pt1 = k;
                }
            }
            if(pt1 == -1)
                continue;
            int pt2 = -1;
            for(int k = 0; k < int(pdata.features.size()); k++)
            {
                if(k == i || k == j || k == pt1)
                    continue;
                double angle = geometry::vector_angle(geometry::toVector(pdata.features[j], pdata.features[i]),
                                                      geometry::toVector(pdata.features[i], pdata.features[k]));
                if(std::abs(angle - geometry::pi/2) < sRectAngleEps)
                {
                    if(pt2 == -1 || geometry::dist(pdata.features[i], pdata.features[k]) >
                            geometry::dist(pdata.features[i], pdata.features[pt2]))
                        pt2 = k;
                }
            }
            if(pt2 != -1)
            {
                double angle = geometry::vector_angle(geometry::toVector(pdata.features[i], pdata.features[pt2]),
                                                      geometry::toVector(pdata.features[pt2], pdata.features[pt1]));
                if(std::abs(angle - geometry::pi / 2) > sRectAngleEps)
                    continue;
                angle = geometry::vector_angle(geometry::toVector(pdata.features[j], pdata.features[pt1]),
                                               geometry::toVector(pdata.features[pt1], pdata.features[pt2]));
                if(std::abs(angle - geometry::pi / 2) > sRectAngleEps)
                    continue;


                if(geometry::vector_len(geometry::toVector(pdata.features[i], pdata.features[j])) *
                        geometry::vector_len(geometry::toVector(pdata.features[j], pdata.features[pt1])) >=
                        pdata.area * 0.7)
                    return true;

            }
        }
    }
    return false;
}

double PuzzleCutter::puzzleAngle(const PuzzleCutter::PuzzleData &pdata)
{
    double ret = geometry::vector_angle(geometry::toVector(pdata.pillarLines[2].second,
                                        pdata.pillarLines[2].first)) / geometry::pi * 180;
    if(ret >= 180.0)
        ret -= 180.0;
    if(ret >= 90.0)
        ret = ret-180.0;
    return ret;
}

void PuzzleCutter::rotateImg(cv::Mat &src, double angle, cv::Mat &dst)
{
    cv::Point2f pt(src.cols / 2, src.rows/2);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
}

void PuzzleCutter::setBackgroundThreshold(int th)
{
    sBackgroundThreshold = th;
}

void PuzzleCutter::setRectAngleEps(double eps)
{
    sRectAngleEps = eps;
}

void PuzzleCutter::setFeatureWindowSize(int sz)
{
    sFeatureWindowSize = sz;
}

void PuzzleCutter::setErosionSize(int sz)
{
    sErosionSize = sz;
}









