// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2014, Oracle and/or its affiliates.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#include <iostream>

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE test_distance
#endif

#include <boost/test/included/unit_test.hpp>

#include "test_distance1.hpp"


typedef bg::model::point<double,2,bg::cs::cartesian>  point_type;
typedef bg::model::multi_point<point_type>            multi_point_type;
typedef bg::model::point<double,3,bg::cs::cartesian>  point_type_3d;
typedef bg::model::segment<point_type>                segment_type;
typedef bg::model::linestring<point_type>             linestring_type;
typedef bg::model::multi_linestring<linestring_type>  multi_linestring_type;
typedef bg::model::polygon<point_type, false>         polygon_type;
typedef bg::model::multi_polygon<polygon_type>        multi_polygon_type;
typedef bg::model::box<point_type>                    box_type;
typedef bg::model::box<point_type_3d>                 box_type_3d;

namespace services = bg::strategy::distance::services;
typedef bg::default_distance_result<point_type>::type return_type;

typedef bg::strategy::distance::pythagoras<> point_point_strategy;
typedef bg::strategy::distance::projected_point<> point_segment_strategy;

//===========================================================================
//===========================================================================
//===========================================================================

template<typename Strategy>
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

template<typename Strategy>
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

template<typename Strategy>
void test_distance_linestring_linestring(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "linestring/linestring distance tests" << std::endl;
#endif
    test_distance_of_geometries<linestring_type, linestring_type> tester;

    // MK:: I have removed the following two test cases
    // It is not obvious that linestrings with only one point are valid

    //    tester("linestring(1 1)", "linestring(2 1)", 1, 1, strategy);

    //    tester("linestring(1 1,3 1)", "linestring(2 1)", 0, 0, strategy);

    //    tester("linestring(1 1)", "linestring(0 0,-2 0,2 -2,2 0)",
    //           sqrt(2.0), 2, strategy);

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

template<typename Strategy>
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

template<typename Strategy>
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


template<typename Strategy>
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

template<typename Strategy>
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

template<typename Strategy>
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

template<typename Strategy>
void test_distance_polygon_polygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "polygon/polygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<polygon_type, polygon_type> tester;

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           "polygon((-5 20,5 20,5 25,-5 25,-5 20))",
           10, 100, strategy);

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           "polygon((-5 20,-5 5,5 5,5 20,-5 20))",
           0, 0, strategy);

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           "polygon((-5 20,-5 -20,5 -20,5 20,-5 20))",
           0, 0, strategy);

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10),\
                    (-5 -5,-5 5,5 5,5 -5,-5 -5))",
           "polygon((-1 -1,0 0,-1 0,-1 -1))",
           4, 16, strategy);
}

//===========================================================================
//===========================================================================
//===========================================================================


template<typename Strategy>
void test_distance_point_multipolygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "point/multipolygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<point_type, multi_polygon_type> tester;

    tester("point(0 -20)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((0 22,-1 30,2 40,0 22)))",
           10, 100, strategy);

    tester("point(12 0)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           2, 4, strategy);

    tester("point(0 0)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy);

    tester("point(0 0)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10),\
                         (-5 -5,-5 5,5 5,5 -5,-5 -5)),\
                         ((100 0,101 0,101 1,100 1,100 0)))",
           5, 25, strategy);
}

//===========================================================================
//===========================================================================
//===========================================================================


template<typename Strategy>
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


template<typename Strategy>
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


template<typename Strategy>
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


template<typename Strategy>
void test_distance_polygon_multipolygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "polygon/multipolygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<polygon_type, multi_polygon_type> tester;

    tester("polygon((12 0,14 0,19 0,19.9 -1,12 0))",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0.1, 0.01, strategy, true);

    tester("polygon((19 0,19.9 -1,12 0,20.5 0.5,19 0))",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy, true);
}

//===========================================================================
//===========================================================================
//===========================================================================


template<typename Strategy>
void test_distance_multipolygon_multipolygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multipolygon/multipolygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_polygon_type, multi_polygon_type> tester;

    tester("multipolygon(((12 0,14 0,14 1,12 0)),((18 0,19 0,19.9 -1,18 0)))",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0.1, 0.01, strategy);

    tester("multipolygon(((18 0,19 0,19.9 -1,18 0)),((12 0,14 0,20.5 0.5,12 0)))",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template<typename Strategy>
void test_distance_point_multipoint(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "point/multipoint distance tests" << std::endl;
#endif
    test_distance_of_geometries<point_type, multi_point_type> tester;

    tester("point(1 1)",
           "multipoint(1 1,2 1,2 2,1 2)",
           0, 0, strategy);
    tester("point(1 1)",
           "multipoint(2 2,2 3,3 2,3 3)",
           sqrt(2.0), 2, strategy);
    tester("point(3 0)",
           "multipoint(2 2,2 4,4 2,4 4)",
           sqrt(5.0), 5, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template<typename Strategy>
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


template<typename Strategy>
void test_distance_multipoint_multipoint(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multipoint/multipoint distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_point_type, multi_point_type> tester;

    tester("multipoint(0 0,1 0,0 1,1 1)",
           "multipoint(1 1,2 1,2 2,1 2)",
           0, 0, strategy);
    tester("multipoint(0 0,1 0,0 1,1 1)",
           "multipoint(2 2,2 3,3 2,3 3)",
           sqrt(2.0), 2, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template<typename Strategy>
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


template<typename Strategy>
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


template<typename Strategy>
void test_distance_point_box_2d(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "2D point/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<box_type, point_type> tester;

    // point inside box
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(0 0)", 0, 0, strategy);

    // points on box corners
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(-1 -1)", 0, 0, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(-1 1)", 0, 0, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(1 -1)", 0, 0, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(1 1)", 0, 0, strategy);

    // points on box boundary edges
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(0 -1)", 0, 0, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(-1 0)", 0, 0, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(1 0)", 0, 0, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(0 1)", 0, 0, strategy);

    // points outside box
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(0 4)", 3, 9, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(0.5 4)", 3, 9, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(-0.5 5)", 4, 16, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(3 0.25)", 2, 4, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(-3 -0.25)", 2, 4, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(3 5)", sqrt(20), 20, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(-5 -4)", 5, 25, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(2 -2)", sqrt(2), 2, strategy);
    tester(make_box2d<box_type>(-1, -1, 1, 1),
           "point(-3 4)", sqrt(13), 13, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template<typename Strategy>
void test_distance_segment_box(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "2D segment/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<box_type, segment_type> tester;

    // segments that intersects the box
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

template<typename Strategy>
void test_distance_point_box_3d(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "3D point/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<box_type_3d, point_type_3d> tester;

    // point inside box
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 0 0)", 0, 0, strategy);

    // points on box corners
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 -1 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 -1 1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 1 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 1 1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 -1 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 -1 1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 1 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 1 1)", 0, 0, strategy);

    // points on box boundary edges
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 -1 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 -1 1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 1 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 1 1)", 0, 0, strategy);

    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 0 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 0 1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 0 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 0 1)", 0, 0, strategy);

    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 -1 0)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 1 0)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 -1 0)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 1 0)", 0, 0, strategy);

    // point on box faces
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 0 -1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 0 1)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 -1 0)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 1 0)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-1 0 0)", 0, 0, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(1 0 0)", 0, 0, strategy);

    // points outside box -- closer to box corners
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-2 -3 -4)", sqrt(14), 14, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-2 -3 3)", 3, 9, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-2 5 -2)", sqrt(18), 18, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-2 5 3)", sqrt(21), 21, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(4 -6 -3)", sqrt(38), 38, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(4 -6 4)", sqrt(43), 43, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(4 7 -2)", sqrt(46), 46, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(4 7 8)", sqrt(94), 94, strategy);


    // points closer to box facets
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 0 10)", 9, 81, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 0 -5)", 4, 16, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 7 0)", 6, 36, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 -6 0)", 5, 25, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(4 0 0)", 3, 9, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-3 0 0)", 2, 4, strategy);

    // points closer to box edges
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 -4 -5)", 5, 25, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 -3 6)", sqrt(29), 29, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 2 -7)", sqrt(37), 37, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(0 8 7)", sqrt(85), 85, strategy);

    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-4 0 -4)", sqrt(18), 18, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-3 0 5)", sqrt(20), 20, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(2 0 -6)", sqrt(26), 26, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(8 0 6)", sqrt(74), 74, strategy);

    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-5 -5 0)", sqrt(32), 32, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(-4 6 0)", sqrt(34), 34, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(3 -7 0)", sqrt(40), 40, strategy);
    tester(make_box3d<box_type_3d>(-1, -1, -1, 1, 1, 1),
           "point(9 7 0)", 10, 100, strategy);
}

//===========================================================================
//===========================================================================
//===========================================================================



template<typename Point, typename Strategy>
void test_more_empty_input(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "testing on empty inputs... " << std::flush;
#endif

    typedef bg::model::linestring<Point> Linestring;

    bg::model::linestring<Point> line_empty;
    bg::model::polygon<Point> polygon_empty;
    bg::model::multi_linestring<Linestring> multiline_empty;

    test_empty_input(line_empty, line_empty, strategy);
    test_empty_input(line_empty, multiline_empty, strategy);
    test_empty_input(multiline_empty, line_empty, strategy);
    test_empty_input(line_empty, polygon_empty, strategy);
    test_empty_input(polygon_empty, line_empty, strategy);

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

BOOST_AUTO_TEST_CASE( test_all_polygon_polygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_polygon_polygon(pp_strategy);
    test_distance_polygon_polygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_multipolygon(pp_strategy);
    test_distance_point_multipolygon(ps_strategy);
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

BOOST_AUTO_TEST_CASE( test_all_polygon_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_polygon_multipolygon(pp_strategy);
    test_distance_polygon_multipolygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multipolygon_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multipolygon_multipolygon(pp_strategy);
    test_distance_multipolygon_multipolygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_multipoint )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_multipoint(pp_strategy);
    test_distance_point_multipoint(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_linestring_multipoint )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_linestring_multipoint(pp_strategy);
    test_distance_linestring_multipoint(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multipoint_multipoint )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multipoint_multipoint(pp_strategy);
    test_distance_multipoint_multipoint(ps_strategy);
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

BOOST_AUTO_TEST_CASE( test_all_point_box_2d )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_box_2d(pp_strategy);
    test_distance_point_box_2d(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_box_3d )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_box_3d(pp_strategy);
    test_distance_point_box_3d(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_segment_box )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_segment_box(pp_strategy);
    test_distance_segment_box(ps_strategy);
}


BOOST_AUTO_TEST_CASE( test_all_empty_input )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_more_empty_input<point_type>(pp_strategy);
    test_more_empty_input<point_type>(ps_strategy);
}
