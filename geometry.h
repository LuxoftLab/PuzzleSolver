#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace geometry{
    static const double pi = 3.141592653589793238462643383279;
    static const double eps = 0.1;

	struct Line
	{
        double A, B, C;
        Line(const cv::Point &p1, const cv::Point &p2)
		{
			A = p2.y - p1.y;
			B = p1.x - p2.x;
			C = -(A*p1.x + B*p1.y);
		}
        Line(const cv::Point &p)
		{
			A = p.y;
			B = -p.x;
			C = -(A*p.x + B*p.y);
		}

		void normalize()
		{
			double d = sqrt(A*A + B*B);
			A /= d;
			B /= d;
			C /= d;
		}

        double point_value(const cv::Point &p)
		{
			return A*p.x + B*p.y + C;
		}
	};

    static cv::Point toVector(const cv::Point &p1, const cv::Point &p2)
	{
        return cv::Point(p2.x - p1.x, p2.y - p1.y);
	}

    static double scalar_product(const cv::Point &p1, const cv::Point &p2)
	{
		return p1.x * p2.x + p1.y * p2.y;
	}

    static double vector_product(const cv::Point &p1, const cv::Point &p2)
	{
		return p1.x * p2.y - p2.x * p1.y;
    }

    static double vector_len(const cv::Point &p)
	{
		return sqrt(scalar_product(p, p));
	}

    static double dist(const cv::Point &p1, const cv::Point &p2)
	{
		return vector_len(p2 - p1);
	}

    static double vector_angle(const cv::Point &v)
	{
		double ret = atan2(v.y, v.x);
		return ret + eps < 0 ? ret + 2 * pi : ret;
	}

    static double vector_angle(const cv::Point &v1, const cv::Point &v2)
	{
		return acos(scalar_product(v1, v2) / vector_len(v1) / vector_len(v2));
	}

    static cv::Point vector_rotate(const cv::Point& p, double angle)
    {
        return cv::Point(cos(angle)*p.x - sin(angle)*p.y, sin(angle)*p.x + cos(angle)*p.y);
    }


    static int sign(double v)
	{
        return v > 0;
	}

    static double polygon_area(const std::vector <cv::Point> &pt)
    {
        double area = 0;
        for (int i = 0; i < int(pt.size()); i++)
            area += vector_product(pt[i], pt[(i + 1) % int(pt.size())]);
        return fabs(area) / 2;
    }

    static bool isInPolygon(const std::vector <std::pair <cv::Point, cv::Point> > &pt, cv::Point p)
    {
        int cnt = 0;
        for (int i = 0; i < int(pt.size()); i++)
            cnt += sign(vector_product(toVector(pt[i].first, pt[i].second), toVector(pt[i].second, p)));
        return cnt == 0 || cnt == int(pt.size());
    }
}

#endif // GEOMETRY_H
