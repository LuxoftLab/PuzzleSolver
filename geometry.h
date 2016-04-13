#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace geometry{
    static const double pi = 3.141592653589793238462643383279;
    static const double eps = 1e-9;

	struct Line
	{
        double A, B, C;
        Line(const cv::Point &p1, const cv::Point &p2);
        Line(const cv::Point &p);
        void normalize();
        double point_value(const cv::Point &p);
	};

    cv::Point toVector(const cv::Point &p1, const cv::Point &p2);
    double scalar_product(const cv::Point &p1, const cv::Point &p2);
    double vector_product(const cv::Point &p1, const cv::Point &p2);
    double vector_len(const cv::Point &p);
    double dist(const cv::Point &p1, const cv::Point &p2);
    double vector_angle(const cv::Point &v);
    double angle_fix(double angle);
    double vector_angle(const cv::Point &v1, const cv::Point &v2);
    cv::Point vector_rotate(const cv::Point& p, double angle);
    int sign(double v);
    double polygon_area(const std::vector <cv::Point> &pt);
    bool isInPolygon(const std::vector <std::pair <cv::Point, cv::Point> > &pt, cv::Point p);
}

#endif // GEOMETRY_H
