// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE test_distance_pointlike_linear
#endif

#include <boost/test/included/unit_test.hpp>

#include "test_distance_common.hpp"


typedef bg::model::point<double,2,bg::cs::cartesian>  point_type;
typedef bg::model::multi_point<point_type>            multi_point_type;
typedef bg::model::segment<point_type>                segment_type;
typedef bg::model::linestring<point_type>             linestring_type;
typedef bg::model::multi_linestring<linestring_type>  multi_linestring_type;

namespace services = bg::strategy::distance::services;
typedef bg::default_distance_result<point_type>::type return_type;

typedef bg::strategy::distance::pythagoras<> point_point_strategy;
typedef bg::strategy::distance::projected_point<> point_segment_strategy;

//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_point_segment(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "point/segment distance tests" << std::endl;
#endif
    test_distance_of_geometries<point_type, segment_type> tester;

    tester("point(0 0)", "segment(2 0,3 0)", 2, 4, strategy);
    tester("point(2.5 3)", "segment(2 0,3 0)", 3, 9, strategy);
    tester("point(2 0)", "segment(2 0,3 0)", 0, 0, strategy);
    tester("point(3 0)", "segment(2 0,3 0)", 0, 0, strategy);
    tester("point(2.5 0)", "segment(2 0,3 0)", 0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_point_linestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "point/linestring distance tests" << std::endl;
#endif
    test_distance_of_geometries<point_type, linestring_type> tester;

    tester("point(0 0)", "linestring(2 0,3 0)", 2, 4, strategy);
    tester("point(2.5 3)", "linestring(2 0,3 0)", 3, 9, strategy);
    tester("point(2 0)", "linestring(2 0,3 0)", 0, 0, strategy);
    tester("point(3 0)", "linestring(2 0,3 0)", 0, 0, strategy);
    tester("point(2.5 0)", "linestring(2 0,3 0)", 0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_point_multilinestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "point/multilinestring distance tests" << std::endl;
#endif
    test_distance_of_geometries<point_type, multi_linestring_type> tester;

    tester("point(0 0)", "multilinestring((-5 0,-3 0),(2 0,3 0))",
           2, 4, strategy);
    tester("point(2.5 3)", "multilinestring((-5 0,-3 0),(2 0,3 0))",
           3, 9, strategy);
    tester("point(2 0)", "multilinestring((-5 0,-3 0),(2 0,3 0))",
           0, 0, strategy);
    tester("point(3 0)", "multilinestring((-5 0,-3 0),(2 0,3 0))",
           0, 0, strategy);
    tester("point(2.5 0)", "multilinestring((-5 0,-3 0),(2 0,3 0))",
           0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_linestring_multipoint(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "linestring/multipoint distance tests" << std::endl;
#endif
    test_distance_of_geometries<linestring_type, multi_point_type> tester;

    tester("linestring(2 0,0 2)",
           "multipoint(0 0,1 0,0 1,1 1)",
           0, 0, strategy);
    tester("linestring(4 0,0 4)",
           "multipoint(0 0,1 0,0 1,1 1)",
           sqrt(2.0), 2, strategy);
    tester("linestring(1 1,2 2)",
           "multipoint(0 0,1 0,0 1,1 1)",
           0, 0, strategy);
    tester("linestring(3 3,4 4)",
           "multipoint(0 0,1 0,0 1,1 1)",
           sqrt(8.0), 8, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multipoint_multilinestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multipoint/multilinestring distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_point_type, multi_linestring_type> tester;

    tester("multipoint(0 0,1 0,0 1,1 1)",
           "multilinestring((2 0,0 2),(2 2,3 3))",
           0, 0, strategy);
    tester("multipoint(0 0,1 0,0 1,1 1)",
           "multilinestring((3 0,0 3),(4 4,5 5))",
           0.5 * sqrt(2.0), 0.5, strategy);
    tester("multipoint(0 0,1 0,0 1,1 1)",
           "multilinestring((4 4,5 5),(1 1,2 2))",
           0, 0, strategy);
    tester("multipoint(0 0,1 0,0 1,1 1)",
           "multilinestring((3 3,4 4),(4 4,5 5))",
           sqrt(8.0), 8, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multipoint_segment(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multipoint/segment distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_point_type, segment_type> tester;

    tester("multipoint(0 0,1 0,0 1,1 1)",
           make_segment<segment_type>(2, 0, 0, 2),
           0, 0, strategy);
    tester("multipoint(0 0,1 0,0 1,1 1)",
           make_segment<segment_type>(4, 0, 0, 4),
           sqrt(2.0), 2, strategy);
    tester("multipoint(0 0,1 0,0 1,1 1)",
           make_segment<segment_type>(1, 1, 2, 2),
           0, 0, strategy);
    tester("multipoint(0 0,1 0,0 1,1 1)",
           make_segment<segment_type>(3, 3, 4, 4),
           sqrt(8.0), 8, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Point, typename Strategy>
void test_more_empty_input_pointlike_linear(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "testing on empty inputs... " << std::flush;
#endif
    bg::model::linestring<Point> line_empty;
    bg::model::multi_point<Point> multipoint_empty;
    bg::model::multi_linestring<bg::model::linestring<Point> > multiline_empty;

    test_empty_input(line_empty, multipoint_empty, strategy);
    test_empty_input(multiline_empty, multipoint_empty, strategy);

#ifdef GEOMETRY_TEST_DEBUG
    std::cout << "done!" << std::endl;
#endif
}


//===========================================================================
//===========================================================================
//===========================================================================

BOOST_AUTO_TEST_CASE( test_all_point_segment )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_segment(pp_strategy);
    test_distance_point_segment(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_linestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_linestring(pp_strategy);
    test_distance_point_linestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_multilinestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_multilinestring(pp_strategy);
    test_distance_point_multilinestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_linestring_multipoint )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_linestring_multipoint(pp_strategy);
    test_distance_linestring_multipoint(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multipoint_multilinestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multipoint_multilinestring(pp_strategy);
    test_distance_multipoint_multilinestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multipoint_segment )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multipoint_segment(pp_strategy);
    test_distance_multipoint_segment(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_empty_input_pointlike_linear )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_more_empty_input_pointlike_linear<point_type>(pp_strategy);
    test_more_empty_input_pointlike_linear<point_type>(ps_strategy);
}
