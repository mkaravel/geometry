// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_TEST_RANDOM_POINT_GENERATOR_HPP
#define BOOST_GEOMETRY_TEST_RANDOM_POINT_GENERATOR_HPP

#include <cstdlib>
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

    point apply(double xmin, double xmax, double ymin, double ymax) const
    {
        double x = random_coordinate(xmin, xmax);
        double y = random_coordinate(ymin, ymax);
        return std::make_pair(x, y);
    }

    point apply(double min, double max) const
    {
        return apply(min, max, min, max);
    }

    point operator*() const
    {
        return apply(m_xmin, m_xmax, m_ymin, m_ymax);
    }

    bool inside_box(point const& q) const
    {
        return
            m_xmin <= q.first && q.first <= m_xmax &&
            m_ymin <= q.second && q.second <= m_ymax;
    }
};


#endif // BOOST_GEOMETRY_TEST_RANDOM_POINT_GENERATOR_HPP
