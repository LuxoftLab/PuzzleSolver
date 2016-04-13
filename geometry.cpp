#include "geometry.h"




geometry::Line::Line(const cv::Point &p1, const cv::Point &p2)
{
    A = p2.y - p1.y;
    B = p1.x - p2.x;
    C = -(A*p1.x + B*p1.y);
}

geometry::Line::Line(const cv::Point &p)
{
    A = p.y;
    B = -p.x;
    C = -(A*p.x + B*p.y);
}

void geometry::Line::normalize()
{
    double d = sqrt(A*A + B*B);
    A /= d;
    B /= d;
    C /= d;
}

double geometry::Line::point_value(const cv::Point &p)
{
    return A*p.x + B*p.y + C;
}


cv::Point geometry::toVector(const cv::Point &p1, const cv::Point &p2)
{
    return cv::Point(p2.x - p1.x, p2.y - p1.y);
}


double geometry::scalar_product(const cv::Point &p1, const cv::Point &p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}


double geometry::vector_product(const cv::Point &p1, const cv::Point &p2)
{
    return p1.x * p2.y - p2.x * p1.y;
}


double geometry::vector_len(const cv::Point &p)
{
    return sqrt(scalar_product(p, p));
}


double geometry::dist(const cv::Point &p1, const cv::Point &p2)
{
    return vector_len(p2 - p1);
}


double geometry::vector_angle(const cv::Point &v)
{
    double ret = atan2(v.y, v.x);
    return ret + eps < 0 ? ret + 2 * pi : ret;
}


double geometry::angle_fix(double angle)
{
    if(angle >= 180.0)
        angle -= 180.0;
    if(angle >= 90.0)
        angle = angle-180.0;
    return angle;
}


double geometry::vector_angle(const cv::Point &v1, const cv::Point &v2)
{
    return acos(scalar_product(v1, v2) / vector_len(v1) / vector_len(v2));
}


cv::Point geometry::vector_rotate(const cv::Point &p, double angle)
{
    return cv::Point(cos(angle)*p.x - sin(angle)*p.y, sin(angle)*p.x + cos(angle)*p.y);
}


int geometry::sign(double v)
{
    return v > 0;
}


double geometry::polygon_area(const std::vector<cv::Point> &pt)
{
    double area = 0;
    for (int i = 0; i < int(pt.size()); i++)
        area += vector_product(pt[i], pt[(i + 1) % int(pt.size())]);
    return fabs(area) / 2;
}


bool geometry::isInPolygon(const std::vector<std::pair<cv::Point, cv::Point> > &pt, cv::Point p)
{
    int cnt = 0;
    for (int i = 0; i < int(pt.size()); i++)
        cnt += sign(vector_product(toVector(pt[i].first, pt[i].second), toVector(pt[i].second, p)));
    return cnt == 0 || cnt == int(pt.size());
}
