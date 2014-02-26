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
#include <boost/geometry/multi/algorithms/detail/distance/closest_distance.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/closest_distance_rtree_query_it.hpp>

#ifdef BOOST_GEOMETRY_USE_TIMER
#include <boost/timer/timer.hpp>
#endif

namespace bg = ::boost::geometry;

typedef bg::model::d2::point_xy<double> point;
typedef bg::model::linestring<point> linestring;
typedef bg::model::multi_linestring<linestring> multi_linestring;


// this the implementation of the Rabin-like algorithm
typedef bg::detail::distance::closest_distance
<
    multi_linestring, multi_linestring
> CD;



template <typename Linear1, typename Linear2>
struct linear_to_linear
{
    typedef typename bg::strategy::distance::services::default_strategy
    <
        bg::point_tag,
        typename bg::point_type<Linear1>::type,
        typename bg::point_type<Linear2>::type
    >::type PPStrategy;

    typedef typename bg::strategy::distance::services::default_strategy
    <
        bg::segment_tag,
        typename bg::point_type<Linear1>::type,
        typename bg::point_type<Linear2>::type,
        typename bg::cs_tag<typename bg::point_type<Linear1>::type>::type,
        typename bg::cs_tag<typename bg::point_type<Linear2>::type>::type,
        PPStrategy
    >::type Strategy;

    typedef typename bg::detail::distance::linear_to_linear_rtree
    <
        Linear1, Linear2, Strategy, true, true
    > linear_to_linear_rtree;

    static inline typename linear_to_linear_rtree::return_type
    apply(Linear1 const& linear1, Linear2 const& linear2)
    {
        return
            linear_to_linear_rtree::apply(linear1, linear2, Strategy(), true);
    }
};


// this is the R-Tree based algorithm
typedef linear_to_linear<multi_linestring, multi_linestring> CDRT;

// this is the alternative R-Tree based algorithm (uses query iterator)
typedef bg::detail::distance::closest_distance_rtree_query_iterator
<
    multi_linestring, multi_linestring
> CDRTL;



template <typename Multi1, typename Multi2>
struct multi_to_multi
{
    typedef typename bg::strategy::distance::services::default_strategy
    <
        bg::point_tag,
        typename bg::point_type<Multi1>::type,
        typename bg::point_type<Multi2>::type
    >::type Strategy;

    typedef bg::detail::distance::distance_multi_to_multi
    <
        Multi1, Multi2, Strategy
    > multi_to_multi_algo;

    static inline typename multi_to_multi_algo::return_type
    apply(Multi1 const& m1, Multi2 const& m2)
    {        
        return multi_to_multi_algo::apply(m1, m2, Strategy());
    }
};


// the old BG implementation
typedef multi_to_multi<multi_linestring, multi_linestring> OldBG;



int main(int argc, char** argv)
{
    multi_linestring mls1, mls2;

    if ( argc == 1 ) { return 0; }

    std::ifstream ifs(argv[1]);
    assert( ifs );

    std::size_t n_red, n_blue;
    double max_length;

    ifs >> n_red >> n_blue >> max_length;

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
    std::cout << "prescribed max length: " << max_length << std::endl;
    std::cout << std::endl << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "intersects (get_turns) = "
                  << (bg::intersects(mls1, mls2) ? "YES" : "NO") << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "closest-distance function (R-tree) = "
                  << CDRT::apply(mls1, mls2) << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "distance (Rabin-like) = "
                  << CD::apply(mls1, mls2) << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "closest-distance function (R-tree query iterator) = "
                  << CDRTL::apply(mls1, mls2) << std::endl;
    }
    std::cout << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        std::cout << "distance (old BG) = "
                  << OldBG::apply(mls1, mls2) << std::endl;
    }

    return 0;
}
