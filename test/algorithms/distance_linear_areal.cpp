// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE test_distance_linear_areal
#endif

#include <boost/test/included/unit_test.hpp>

#include "test_distance_common.hpp"


typedef bg::model::point<double,2,bg::cs::cartesian>  point_type;
//typedef bg::model::multi_point<point_type>            multi_point_type;
//typedef bg::model::point<double,3,bg::cs::cartesian>  point_type_3d;
typedef bg::model::segment<point_type>                segment_type;
typedef bg::model::linestring<point_type>             linestring_type;
typedef bg::model::multi_linestring<linestring_type>  multi_linestring_type;
typedef bg::model::polygon<point_type, false>         polygon_type;
typedef bg::model::multi_polygon<polygon_type>        multi_polygon_type;
typedef bg::model::box<point_type>                    box_type;
//typedef bg::model::box<point_type_3d>                 box_type_3d;

namespace services = bg::strategy::distance::services;
typedef bg::default_distance_result<point_type>::type return_type;

typedef bg::strategy::distance::pythagoras<> point_point_strategy;
typedef bg::strategy::distance::projected_point<> point_segment_strategy;

//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_segment_polygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "segment/polygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<segment_type, polygon_type> tester;

    tester(make_segment<segment_type>(-1, 20, 1, 20),
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           10, 100, strategy);

    tester(make_segment<segment_type>(1, 20, 2, 40),
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           10, 100, strategy);

    tester(make_segment<segment_type>(-1, 20, -1, 5),
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy);

    tester(make_segment<segment_type>(-1, 20, -1, -20),
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_linestring_polygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "linestring/polygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<linestring_type, polygon_type> tester;

    tester("linestring(-1 20,1 20,1 30)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           10, 100, strategy, true);
  
    tester("linestring(-1 20,1 20,1 5)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy, true);

    tester("linestring(-1 20,1 20,1 -20)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy, true);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multilinestring_polygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multilinestring/polygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_linestring_type, polygon_type> tester;

    tester("multilinestring((-100 -100,-90 -90),(-1 20,1 20,1 30))",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           10, 100, strategy, true);
  
    tester("multilinestring((-1 20,1 20,1 30),(-1 20,1 20,1 5))",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy, true);

    tester("multilinestring((-1 20,1 20,1 30),(-1 20,1 20,1 -20))",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy, true);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_segment_multipolygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "segment/multipolygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<segment_type, multi_polygon_type> tester;

    tester(make_segment<segment_type>(-1, 20, 1, 20),
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((0 22,-1 30, 2 40,0 22)))",
           2, 4, strategy);

    tester(make_segment<segment_type>(12, 0, 14, 0),
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           2, 4, strategy);

    tester(make_segment<segment_type>(12, 0, 20.5, 0.5),
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy);

    tester(make_segment<segment_type>(12, 0, 50, 0),
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_linestring_multipolygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "linestring/multipolygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<linestring_type, multi_polygon_type> tester;

    tester("linestring(-1 20,1 20)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((0 22,-1 30, 2 40,0 22)))",
           2, 4, strategy, true);
    
    tester("linestring(12 0,14 0)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           2, 4, strategy, true);

    tester("linestring(12 0,20.5 0.5)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy, true);

    tester("linestring(12 0,50 0)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy, true);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multilinestring_multipolygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multilinestring/multipolygon distance tests" << std::endl;
#endif
    test_distance_of_geometries
        <
            multi_linestring_type, multi_polygon_type
        > tester;

    tester("multilinestring((12 0,14 0),(19 0,19.9 -1))",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10)))",
           0.1, 0.01, strategy, true);

    tester("multilinestring((19 0,19.9 -1),(12 0,20.5 0.5))",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy, true);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_segment_box(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "2D segment/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<box_type, segment_type> tester;

    // segments that intersect the box
    tester(make_box2d<box_type>(0, 0, 1, 1),
           make_segment<segment_type>(-1, 0.5, 0.5, 0.75),
           0, 0, strategy);
    tester(make_box2d<box_type>(0, 0, 1, 1),
           make_segment<segment_type>(-1, 0.5, 1.5, 0.75),
           0, 0, strategy);
    tester(make_box2d<box_type>(0, 0, 1, 1),
           make_segment<segment_type>(0.5, -1, 0.5, 2),
           0, 0, strategy);
    tester(make_box2d<box_type>(0, 0, 1, 1),
           make_segment<segment_type>(1, 1, 1.5, 0.75),
           0, 0, strategy);
    tester(make_box2d<box_type>(0, 0, 1, 1),
           make_segment<segment_type>(2, 0, 0, 2),
           0, 0, strategy);
    
    // segment that has closest point on box boundary
    tester(make_box2d<box_type>(0, 0, 1, 1),
           make_segment<segment_type>(4, 0.5, 5, 0.75),
           3, 9, strategy);

    // segment that has closest point on box corner
    tester(make_box2d<box_type>(0, 0, 1, 1),
           make_segment<segment_type>(4, 0, 0, 4),
           sqrt(2), 2, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_linestring_box(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "linestring/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<linestring_type, box_type> tester;

    // linestrings that intersect the box
    tester("linestring(-1 0.5,0.5 0.75)",
           make_box2d<box_type>(0, 0, 1, 1),
           0, 0, strategy);
    tester("linestring(-1 0.5,1.5 0.75)",
           make_box2d<box_type>(0, 0, 1, 1),
           0, 0, strategy);
    
    // linestring that has closest point on box boundary
    tester("linestring(4 0.5,5 0.75)",
           make_box2d<box_type>(0, 0, 1, 1),
           3, 9, strategy);

    // linestring that has closest point on box corner
    tester("linestring(4 0,0 4)",
           make_box2d<box_type>(0, 0, 1, 1),
           sqrt(2), 2, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multilinestring_box(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multilinestring/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_linestring_type, box_type> tester;

    // multilinestring that intersects the box
    tester("multilinestring((-1 0.5,0.5 0.75),(4 0.5,5 0.75))",
           make_box2d<box_type>(0, 0, 1, 1),
           0, 0, strategy);
    
    // multilinestring that has closest point on box boundary
    tester("multilinestring((4 0.5,5 0.75))",
           make_box2d<box_type>(0, 0, 1, 1),
           3, 9, strategy);

    // multilinestring that has closest point on box corner
    tester("multilinestring((5 0,0 5),(4 0,0 4))",
           make_box2d<box_type>(0, 0, 1, 1),
           sqrt(2), 2, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Point, typename Strategy>
void test_more_empty_input_linear_areal(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "testing on empty inputs... " << std::flush;
#endif
    bg::model::linestring<Point> line_empty;
    bg::model::polygon<Point> polygon_empty;
    bg::model::multi_linestring<bg::model::linestring<Point> > multiline_empty;
    bg::model::multi_polygon<bg::model::polygon<Point> > multipolygon_empty;

    test_empty_input(line_empty, polygon_empty, strategy);
    test_empty_input(line_empty, multipolygon_empty, strategy);
    //    test_empty_input(multiline_empty, polygon_empty, strategy);
    test_empty_input(multiline_empty, multipolygon_empty, strategy);

#ifdef GEOMETRY_TEST_DEBUG
    std::cout << "done!" << std::endl;
#endif
}


//===========================================================================
//===========================================================================
//===========================================================================

BOOST_AUTO_TEST_CASE( test_all_segment_polygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_segment_polygon(pp_strategy);
    test_distance_segment_polygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_linestring_polygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_linestring_polygon(pp_strategy);
    test_distance_linestring_polygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multilinestring_polygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multilinestring_polygon(pp_strategy);
    test_distance_multilinestring_polygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_segment_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_segment_multipolygon(pp_strategy);
    test_distance_segment_multipolygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_linestring_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_linestring_multipolygon(pp_strategy);
    test_distance_linestring_multipolygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multilinestring_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multilinestring_multipolygon(pp_strategy);
    test_distance_multilinestring_multipolygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_segment_box )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_segment_box(pp_strategy);
    test_distance_segment_box(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_linestring_box )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_linestring_box(pp_strategy);
    test_distance_linestring_box(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multilinestring_box )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multilinestring_box(pp_strategy);
    test_distance_multilinestring_box(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_empty_input_linear_areal )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_more_empty_input_linear_areal<point_type>(pp_strategy);
    test_more_empty_input_linear_areal<point_type>(ps_strategy);
}
