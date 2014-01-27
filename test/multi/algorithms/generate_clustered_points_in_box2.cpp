// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cassert>
#include <utility>

struct random_point_generator
{
    typedef std::pair<double,double> point;

    double m_xmin, m_xmax, m_ymin, m_ymax;

    random_point_generator(double xmin = 0, double xmax = 1,
                           double ymin = 0, double ymax = 1,
                           int seed = 0)
        : m_xmin(xmin), m_xmax(xmax), m_ymin(ymin), m_ymax(ymax)
    {
        srand(seed);
    }

    double random_coordinate(double fmin, double fmax) const
    {
        double f = static_cast<double>(rand()) / RAND_MAX;
        return fmin + f * (fmax - fmin);
    }

    point apply(double min, double max) const
    {
        double x = random_coordinate(min, max);
        double y = random_coordinate(min, max);
        return std::make_pair(x, y);
    }

    point apply(double xmin, double xmax, double ymin, double ymax) const
    {
        double x = random_coordinate(xmin, xmax);
        double y = random_coordinate(ymin, ymax);
        return std::make_pair(x, y);
    }

    point operator*() const
    {
        double x = random_coordinate(m_xmin, m_xmax);
        double y = random_coordinate(m_ymin, m_ymax);
        return std::make_pair(x, y);
    }
};


int main(int argc, char** argv)
{
    assert( argc >= 4 );
    int n = atoi(argv[1]);
    int m = atoi(argv[2]);

    std::ofstream ofs( argv[3] );
    assert( ofs );

    ofs.precision(16);

    ofs << n << " " << m << std::endl;

    random_point_generator g;
    for (int i = 0; i < n / 2; ++i)
    {
        random_point_generator::point p = g.apply(0, 0.01);
        ofs << p.first << " " << p.second << std::endl;
    }
    for (int i = n / 2; i < n; ++i)
    {
        random_point_generator::point p = g.apply(0.99, 1, 0, 0.01);
        ofs << p.first << " " << p.second << std::endl;
    }
    for (int i = 0; i < m / 2; ++i)
    {
        random_point_generator::point p = g.apply(0.99, 1);
        ofs << p.first << " " << p.second << std::endl;
    }
    for (int i = m / 2; i < m; ++i)
    {
        random_point_generator::point p = g.apply(0, 0.01, 0.99, 1);
        ofs << p.first << " " << p.second << std::endl;
    }

    ofs.close();

    return 0;
}
