// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_TEST_RANDOM_SEGMENT_GENERATOR_HPP
#define BOOST_GEOMETRY_TEST_RANDOM_SEGMENT_GENERATOR_HPP

#include "random_point_generator.hpp"
#include <cmath>

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
                             double xmin = 0, double xmax = 1,
                             double ymin = 0, double ymax = 1,
                             int seed = 0)
        : max_length(max_length), g(xmin, xmax, ymin, ymax, seed)
    {}



    segment apply(double xmin, double xmax, double ymin, double ymax) const
    {
        point p = g.apply(xmin, xmax, ymin, ymax);
        point q;

        if ( max_length >= 0 )
        {
            do
            {
                double dx = g.random_coordinate(-max_length, max_length);
                double dy = g.random_coordinate(-max_length, max_length);
                q = point(p.first + dx, p.second + dy);
            }
            while ( distance(p, q) > max_length || !g.inside_box(q) );
        }
        else
        {
            q = g.apply(xmin, xmax, ymin, ymax);
        }

        return std::make_pair(p, q);
    }


    segment apply(double min, double max) const
    {
        return apply(min, max, min, max);
    }


    segment operator*() const
    {
        return apply(g.m_xmin, g.m_xmax, g.m_ymin, g.m_ymax);
    }
};



#endif // BOOST_GEOMETRY_TEST_RANDOM_SEGMENT_GENERATOR_HPP
