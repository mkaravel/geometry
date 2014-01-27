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
#include <cmath>

struct random_point_generator
{
    typedef std::pair<double,double> point;

    double fmin, fmax;

    random_point_generator(double fmin = 0, double fmax = 1, int seed = 0)
        : fmin(fmin), fmax(fmax)
    {
        srand(seed);
    }

    double random_coordinate() const
    {
        double f = static_cast<double>(rand()) / RAND_MAX;
        return fmin + f * (fmax - fmin);
    }

    double random_coordinate(double fmin1, double fmax1) const
    {
        double f = static_cast<double>(rand()) / RAND_MAX;
        return fmin1 + f * (fmax1 - fmin1);
    }

    point operator*() const
    {
        double x = random_coordinate();
        double y = random_coordinate();
        return std::make_pair(x, y);
    }
};


struct random_segment_generator
{
    typedef random_point_generator::point point;
    typedef std::pair<point,point> segment;

    double distance(point const& p1, point const& p2) const
    {
        double dx = p1.first - p2.first;
        double dy = p1.second - p2.second;
        return std::sqrt( dx * dx + dy * dy );
    }

    double max_length;
    random_point_generator g;

    random_segment_generator(double max_length = -1,
                             double fmin = 0, double fmax = 1,
                             int seed = 0)
        : max_length(max_length), g(fmin, fmax, seed)
    {}

    segment operator*() const
    {

        point p = *g;
        point q;

        if ( max_length >= 0 )
        {
            do
            {
                double dx = g.random_coordinate(0, max_length);
                double dy = g.random_coordinate(0, max_length);
                q = point(p.first + dx, p.second + dy);
            }
            while ( distance(p, q) > max_length );
        }
        else
        {
            q = *g;
        }

        return std::make_pair(p, q);
    }
};


int main(int argc, char** argv)
{
    assert( argc >= 4 );
    int n = atoi(argv[1]);
    int m = atoi(argv[2]);

    std::ofstream ofs( argv[3] );
    assert( ofs );

    double max_length = -1;
    if ( argc > 4 )
    {
        max_length = atof(argv[4]);
        std::cout << "maximum length of generated segments: "
                  << max_length << std::endl;
    }

    ofs.precision(16);

    ofs << n << " " << m << std::endl;

    random_segment_generator g(max_length);
    for (int i = 0; i < n + m; ++i)
    {
        random_segment_generator::segment s = *g;
        ofs << s.first.first << " " << s.first.second << " "
            << s.second.first << " " << s.second.second << std::endl;
    }

    ofs.close();

    return 0;
}
