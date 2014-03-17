// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE test_distance_linear_linear
#endif

#include <boost/test/included/unit_test.hpp>

#include "test_distance_common.hpp"


typedef bg::model::point<double,2,bg::cs::cartesian>  point_type;
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
void test_distance_segment_segment(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "segment/segment distance tests" << std::endl;
#endif
    test_distance_of_geometries<segment_type, segment_type> tester;

    tester(make_segment<segment_type>(0, 0, 10, 0),
           make_segment<segment_type>(4, 2, 4, 0.5),
           return_type(0.5), return_type(0.25), strategy);

    tester(make_segment<segment_type>(0, 0, 10, 0),
           make_segment<segment_type>(4, 2, 4, -0.5),
           return_type(0), return_type(0), strategy);

    tester(make_segment<segment_type>(0, 0, 10, 0),
           make_segment<segment_type>(4, 2, 0, 0),
           return_type(0), return_type(0), strategy);

    tester(make_segment<segment_type>(0, 0, 10, 0),
           make_segment<segment_type>(-2, 3, 1, 2),
           return_type(2), return_type(4), strategy);
}

//===========================================================================
//===========================================================================
//===========================================================================

template <typename Strategy>
void test_distance_segment_linestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "segment/linestring distance tests" << std::endl;
#endif
    test_distance_of_geometries<segment_type, linestring_type> tester;

    tester(make_segment<segment_type>(-1, -1, -2, -2),
           "linestring(2 1,1 2,4 0)",
           sqrt(12.5), 12.5, strategy);

    tester(make_segment<segment_type>(1, 1, 2, 2),
           "linestring(2 1,1 2,4 0)",
           0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================

template <typename Strategy>
void test_distance_linestring_linestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "linestring/linestring distance tests" << std::endl;
#endif
    test_distance_of_geometries<linestring_type, linestring_type> tester;

    // MK:: I have removed the following three test cases
    // It is not obvious that linestrings with only one point are valid

    //    tester("linestring(1 1)", "linestring(2 1)", 1, 1, strategy);

    //    tester("linestring(1 1,3 1)", "linestring(2 1)", 0, 0, strategy);

    //    tester("linestring(1 1)", "linestring(0 0,-2 0,2 -2,2 0)",
    //           sqrt(2.0), 2, strategy);

    tester("linestring(1 1,1 1)", "linestring(2 1,2 1)", 1, 1, strategy);

    tester("linestring(1 1,1 1,1 1)", "linestring(2 1,2 1,2 1,2 1)",
           1, 1, strategy);

    tester("linestring(1 1,3 1)", "linestring(2 1,2 1)", 0, 0, strategy);

    tester("linestring(1 1,1 1)", "linestring(0 0,-2 0,2 -2,2 0)",
           sqrt(2.0), 2, strategy);

    tester("linestring(1 1,3 1)", "linestring(2 1, 4 1)", 0, 0, strategy);

    tester("linestring(1 1,2 2,3 3)", "linestring(2 1,1 2,4 0)",
           0, 0, strategy);

    tester("linestring(1 1,2 2,3 3)", "linestring(1 0,2 -1,4 0)",
           1, 1, strategy);

    tester("linestring(1 1,2 2,3 3)", "linestring(1 -10,2 0,2.1 -10,4 0)",
           sqrt(2.0), 2, strategy);

    tester("linestring(1 1,2 2,3 3)", "linestring(1 -10,2 1.9,2.1 -10,4 0)",
           sqrt(0.005), 0.005, strategy);

    tester("linestring(1 1,1 2)", "linestring(0 0,-2 0,2 -2,2 0)",
           sqrt(2.0), 2, strategy);
}

//===========================================================================
//===========================================================================
//===========================================================================

template <typename Strategy>
void test_distance_segment_multilinestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "segment/multilinestring distance tests" << std::endl;
#endif
    test_distance_of_geometries<segment_type, multi_linestring_type> tester;

    tester(make_segment<segment_type>(-1, -1, -2, -2),
           "multilinestring((2 1,1 2),(4 0,4 10))",
           sqrt(12.5), 12.5, strategy);

    tester(make_segment<segment_type>(1, 1, 2, 2),
           "multilinestring((2 1,1 2),(4 0,4 10))",
           0, 0, strategy);
}

//===========================================================================
//===========================================================================
//===========================================================================

template <typename Strategy>
void test_distance_linestring_multilinestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "linestring/multilinestring distance tests" << std::endl;
#endif

    test_distance_of_geometries<linestring_type, multi_linestring_type> tester;

    tester("linestring(1 1,2 2,3 3)",
           "multilinestring((2 1,1 2,4 0),(1 -10,2 1.9,2.1 -10,4 0))",
           0, 0, strategy, true);

    tester("linestring(1 1,2 2,3 3)",
           "multilinestring((1 -10,2 0,2.1 -10,4 0),(1 -10,2 1.9,2.1 -10,4 0))",
           sqrt(0.005), 0.005, strategy, true);
}

//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multilinestring_multilinestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multilinestring/multilinestring distance tests" << std::endl;
#endif

    test_distance_of_geometries
        <
            multi_linestring_type, multi_linestring_type
        > tester;

    tester("multilinestring((0 0,0 1,1 1),(10 0,11 1,12 2))",
           "multilinestring((0.5 0.5,0.75 0.75),(11 0,11 7))",
           0, 0, strategy);

    tester("multilinestring((0 0,0 1,1 1),(10 0,11 1,12 2))",
           "multilinestring((0.5 0.5,0.75 0.75),(11 0,11 0.9))",
           sqrt(0.005), 0.005, strategy);

    tester("multilinestring((0 0,0 1,1 1),(10 0,11 1,12 2))",
           "multilinestring((0.5 0.5,0.75 0.75),(11.1 0,11.1 0.9))",
           sqrt(0.02), 0.02, strategy);
}



//===========================================================================
//===========================================================================
//===========================================================================



template <typename Point, typename Strategy>
void test_more_empty_input_linear_linear(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "testing on empty inputs... " << std::flush;
#endif
    bg::model::linestring<Point> line_empty;
    bg::model::multi_linestring<bg::model::linestring<Point> > multiline_empty;

    test_empty_input(line_empty, line_empty, strategy);
    test_empty_input(line_empty, multiline_empty, strategy);
    test_empty_input(multiline_empty, multiline_empty, strategy);

#ifdef GEOMETRY_TEST_DEBUG
    std::cout << "done!" << std::endl;
#endif
}


//===========================================================================
//===========================================================================
//===========================================================================

BOOST_AUTO_TEST_CASE( test_all_segment_segment )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_segment_segment(pp_strategy);
    test_distance_segment_segment(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_segment_linestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_segment_linestring(pp_strategy);
    test_distance_segment_linestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_linestring_linestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_linestring_linestring(pp_strategy);
    test_distance_linestring_linestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_segment_multilinestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_segment_multilinestring(pp_strategy);
    test_distance_segment_multilinestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_linestring_multilinestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_linestring_multilinestring(pp_strategy);
    test_distance_linestring_multilinestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multilinestring_multilinestring )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multilinestring_multilinestring(pp_strategy);
    test_distance_multilinestring_multilinestring(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_empty_input_linear_linear )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_more_empty_input_linear_linear<point_type>(pp_strategy);
    test_more_empty_input_linear_linear<point_type>(ps_strategy);
}
