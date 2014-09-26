// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <cstddef>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <vector>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/pointing_segment.hpp>

#include <boost/geometry/core/point_type.hpp>

#include <boost/geometry/algorithms/num_points.hpp>
#include <boost/geometry/algorithms/num_segments.hpp>

#include <boost/geometry/iterators/segment_iterator.hpp>

#include "get_segments.hpp"

#ifdef BOOST_GEOMETRY_USE_TIMER
#include <boost/timer/timer.hpp>
#endif

#define NUM_ITERATIONS 10

namespace bg = ::boost::geometry;

typedef bg::model::d2::point_xy<double> point_type;
typedef bg::model::polygon<point_type> polygon_type;


template <typename Iterator>
inline void print_range(Iterator first, Iterator beyond)
{
    for (Iterator it = first; it != beyond; ++it)
    {
        std::cout << bg::dsv(*it) << std::endl;
    }
    std::cout << std::endl;
}


template <typename Linestring>
inline std::size_t get_segments(Linestring const& linestring)
{
    typedef typename bg::point_type<Linestring>::type linestring_point_type;
#if defined(USE_REFERRING_SEGMENT)
    typedef typename bg::model::referring_segment
        <
            linestring_point_type const
        > segment_type;
#elif defined(USE_POINTING_SEGMENT)
    typedef typename bg::model::pointing_segment
        <
            linestring_point_type const
        > segment_type;
#else
    typedef typename bg::model::segment<linestring_point_type> segment_type;
#endif

    std::vector<segment_type> segments;
    bg::detail::distance::get_segments
        <
            Linestring, segment_type
        >::apply(linestring, std::back_inserter(segments));

#ifdef PRINT_SEGMENTS
    print_range(segments.begin(), segments.end());
    std::cout << std::endl << std::endl;
#endif

    return segments.size();
}


template <typename Linestring>
inline std::size_t get_segments_via_iterator(Linestring const& linestring)
{
    typedef typename bg::segment_iterator<Linestring> segment_iterator;
    typedef typename std::iterator_traits
        <
            segment_iterator
        >::value_type segment_type;

    std::vector<segment_type> segments;
    std::copy(bg::segments_begin(linestring),
              bg::segments_end(linestring),
              std::back_inserter(segments));

#ifdef PRINT_SEGMENTS
    print_range(segments.begin(), segments.end());
    std::cout << std::endl << std::endl;
#endif

    return segments.size();
}



int main(int argc, char** argv)
{
    polygon_type poly;

    if ( argc == 1 ) { return 0; }

    std::ifstream ifs(argv[1]);
    assert( ifs );

    std::stringstream sstr;

    while ( ifs )
    {
        std::string tmp;
        ifs >> tmp;
        sstr << tmp << " ";
    }

    ifs.close();

    bg::read_wkt(sstr.str(), poly);

    std::cout.precision(16);
    std::cout << "POLY # of points (bg::num_points): "
              << bg::num_points(poly) << std::endl;
    std::cout << "# of segments (bg::num_segments) = "
              << bg::num_segments(poly)
              << std::endl << std::endl;

    std::size_t n_segs;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        for (int i = 0; i < NUM_ITERATIONS; ++i)
        {
            n_segs = get_segments(poly);
        }
    }
    std::cout << "# of segments (get_segments) = "
              << n_segs << std::endl << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        for (int i = 0; i < NUM_ITERATIONS; ++i)
        {
            n_segs = get_segments_via_iterator(poly);
        }
    }
    std::cout << "# of segments (segment_iterator) = "
              << n_segs << std::endl << std::endl;

    return 0;
}
