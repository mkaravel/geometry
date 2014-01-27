// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#include <iostream>
#include <fstream>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/multi/geometries/multi_linestring.hpp>
#include <boost/geometry/multi/algorithms/distance.hpp>
//#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/closest_distance.hpp>

#ifdef BOOST_GEOMETRY_USE_TIMER
#include <boost/timer/timer.hpp>
#endif

namespace bg = ::boost::geometry;

typedef bg::model::d2::point_xy<double> point;
typedef bg::model::linestring<point> linestring;
typedef bg::model::multi_linestring<linestring> multi_linestring;

typedef bg::detail::distance::closest_distance
<
    multi_linestring,multi_linestring
> CD;

int main(int argc, char** argv)
{
    multi_linestring mls1, mls2;

    if ( argc == 1 ) { return 0; }

    std::ifstream ifs(argv[1]);
    assert( ifs );

    std::size_t n_red, n_blue;

    ifs >> n_red >> n_blue;

    for (std::size_t i = 0; i < n_red; ++i)
    {
        double x1, y1, x2, y2;
        ifs >> x1 >> y1 >> x2 >> y2;
        linestring ls;
        ls.push_back( point(x1, y1) );
        ls.push_back( point(x2, y2) );
        mls1.push_back( ls );
    }

    for (std::size_t i = 0; i < n_blue; ++i)
    {
        double x1, y1, x2, y2;
        ifs >> x1 >> y1 >> x2 >> y2;
        linestring ls;
        ls.push_back( point(x1, y1) );
        ls.push_back( point(x2, y2) );
        mls2.push_back( ls );
    }

    std::cout.precision(16);
    std::cout << "MLS1 size: " << boost::size(mls1) << std::endl;
    std::cout << "MLS2 size: " << boost::size(mls2) << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "distance (Rabin-like) = "
                  << CD::apply(mls1, mls2) << std::endl;
    }
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "intersects (get_turns) = "
                  << (bg::intersects(mls1, mls2) ? "YES" : "NO") << std::endl;
    }
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "distance (brute force) = "
                  << bg::distance(mls1, mls2) << std::endl;
    }

    return 0;
}
