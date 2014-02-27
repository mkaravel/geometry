// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>
#include <fstream>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/multi/algorithms/distance.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/bichromatic_closest_pair.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/closest_distance.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/closest_pair.hpp>


#ifdef BOOST_GEOMETRY_USE_TIMER
#include <boost/timer/timer.hpp>
#endif

namespace bg = ::boost::geometry;

typedef bg::model::d2::point_xy<double> point;
typedef bg::model::multi_point<point> multi_point;

// first implementation of the Rabin-like algorithm for points
typedef bg::detail::distance::bichromatic_closest_pair<multi_point,multi_point>
BCP;

// 2nd implementation of the Rabin-like algorithm
typedef bg::detail::distance::closest_distance<multi_point,multi_point>
CD;

// implementation of closest-pair (returns points)
typedef bg::detail::distance::closest_pair<multi_point,multi_point> CP;


template <typename MultiPoint1, typename MultiPoint2>
struct multipoint_to_multipoint
{
    typedef typename bg::strategy::distance::services::default_strategy
    <
        bg::point_tag,
        typename bg::point_type<MultiPoint1>::type,
        typename bg::point_type<MultiPoint2>::type
    >::type Strategy;

    typedef bg::detail::distance::distance_multi_to_multi
    <
        MultiPoint1, MultiPoint2, Strategy
    > multi_to_multi;

    static inline typename multi_to_multi::return_type
    apply(MultiPoint1 const& mp1, MultiPoint2 const& mp2)
    {        
        return multi_to_multi::apply(mp1, mp2, Strategy());
    }
};


// the old BG implementation
typedef multipoint_to_multipoint<multi_point, multi_point> OldBG;


template <typename MultiPoint1, typename MultiPoint2>
struct multipoint_to_multipoint_rtree
{
    typedef typename bg::strategy::distance::services::default_strategy
    <
        bg::point_tag,
        typename bg::point_type<MultiPoint1>::type,
        typename bg::point_type<MultiPoint2>::type
    >::type Strategy;

    typedef bg::detail::distance::multipoint_to_multipoint
    <
        MultiPoint1, MultiPoint2, Strategy
    > multi_to_multi;

    static inline typename multi_to_multi::return_type
    apply(MultiPoint1 const& mp1, MultiPoint2 const& mp2)
    {
        return multi_to_multi::apply(mp1, mp2, Strategy());
    }
};


// the current BG implementation that uses R-Trees
typedef multipoint_to_multipoint_rtree<multi_point, multi_point> CDRT;


int main(int argc, char** argv)
{
    multi_point mp1, mp2;

    mp1.push_back(point(0,0));
    mp1.push_back(point(1,1));
    mp1.push_back(point(0.5,1));
    mp1.push_back(point(1,0));
    mp1.push_back(point(0,1));

    mp2.push_back(point(-1,-1));
    mp2.push_back(point(-2,-2));
    mp2.push_back(point(-0.5,-1));
    mp2.push_back(point(-1,0));
    mp2.push_back(point(0,-1));

    std::cout << "MP1: " << bg::wkt(mp1) << std::endl;
    std::cout << "MP2: " << bg::wkt(mp2) << std::endl;
    std::cout << "distance (Rabin-like) = "
              << BCP::apply(mp1, mp2) << std::endl;
    std::cout << "distance (brute force) = "
              << bg::distance(mp1, mp2) << std::endl;

    if ( argc == 1 ) { return 0; }

    std::ifstream ifs(argv[1]);
    assert( ifs );

    bg::clear(mp1);
    bg::clear(mp2);

    std::size_t n_red, n_blue;

    ifs >> n_red >> n_blue;

    for (std::size_t i = 0; i < n_red; ++i)
    {
        double x, y;
        ifs >> x >> y;
        mp1.push_back( point(x, y) );
    }

    for (std::size_t i = 0; i < n_blue; ++i)
    {
        double x, y;
        ifs >> x >> y;
        mp2.push_back( point(x, y) );
    }

    std::cout.precision(16);
    std::cout << "MP1 size: " << boost::size(mp1) << std::endl;
    std::cout << "MP2 size: " << boost::size(mp2) << std::endl;
#if 0
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "intersects (get_turns) = "
                  << (bg::intersects(mp1, mp2) ? "YES" : "NO") << std::endl;
    }
#endif
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "closest-distance function (R-tree) = "
                  << CDRT::apply(mp1, mp2) << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "bichromatic closest-distance function (Rabin-like) = "
                  << BCP::apply(mp1, mp2) << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "closest distance function (Rabin-like) = "
                  << CD::apply(mp1, mp2) << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        CP::return_type cp = CP::apply(mp1, mp2);
        std::cout << "closest-pair (Rabin-like) = "
                  << bg::distance(*cp.first, *cp.second)
                  << std::endl
                  << bg::wkt(*cp.first)
                  << " " << bg::wkt(*cp.second)
                  << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "distance (old BG) = "
                  << OldBG::apply(mp1, mp2) << std::endl;
    }

    return 0;
}
