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


#include <boost/geometry.hpp>
#include "test_distance1.hpp"


typedef bg::model::point<double,2,bg::cs::cartesian>  point_type;
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
    test_distance_of_segments tester;

    {
        segment_type s1(point_type(0,0), point_type(10,0));
        segment_type s2(point_type(4,2), point_type(4,0.5));

        tester(s1, s2, return_type(0.5), return_type(0.25), strategy);
    }
    {
        segment_type s1(point_type(0,0), point_type(10,0));
        segment_type s2(point_type(4,2), point_type(4,-0.5));

        tester(s1, s2, return_type(0), return_type(0), strategy);
    }

    {
        segment_type s1(point_type(0,0), point_type(10,0));
        segment_type s2(point_type(4,2), point_type(0,0));

        tester(s1, s2, return_type(0), return_type(0), strategy);
    }
    {
        segment_type s1(point_type(0,0), point_type(10,0));
        segment_type s2(point_type(-2,3), point_type(1,2));

        tester(s1, s2, return_type(2), return_type(4), strategy);
    }
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
    test_distance_of_segment_and_geometry<linestring_type> tester;

    {
        segment_type s(point_type(-1,-1), point_type(-2,-2));

        tester(s, "linestring(2 1,1 2,4 0)", sqrt(12.5), 12.5, strategy);
    }
    {
        segment_type s(point_type(1,1), point_type(2,2));
        tester(s, "linestring(2 1,1 2,4 0)", 0, 0, strategy);
    }
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
    test_distance_of_geometries<linestring_type,linestring_type> tester;

    tester("linestring(1 1)","linestring(2 1)", 1, 1, strategy);

    tester("linestring(1 1,3 1)","linestring(2 1, 4 1)", 0, 0, strategy);

    tester("linestring(1 1,3 1)","linestring(2 1)", 0, 0, strategy);

    tester("linestring(1 1,2 2,3 3)","linestring(2 1,1 2,4 0)",
           0, 0, strategy);

    tester("linestring(1 1,2 2,3 3)","linestring(1 0,2 -1,4 0)",
           1, 1, strategy);

    tester("linestring(1 1,2 2,3 3)","linestring(1 -10,2 0,2.1 -10,4 0)",
           sqrt(2.0), 2, strategy);

    tester("linestring(1 1,2 2,3 3)","linestring(1 -10,2 1.9,2.1 -10,4 0)",
           sqrt(0.005), 0.005, strategy);

    tester("linestring(1 1)","linestring(0 0,-2 0,2 -2,2 0)",
           sqrt(2.0), 2, strategy);

    tester("linestring(1 1,1 2)","linestring(0 0,-2 0,2 -2,2 0)",
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
    test_distance_of_segment_and_geometry<multi_linestring_type> tester;
    {

        segment_type s(point_type(-1,-1), point_type(-2,-2));
      
        tester(s, "multilinestring((2 1,1 2),(4 0,4 10))",
               sqrt(12.5), 12.5, strategy);
    }
    {
        segment_type s(point_type(1,1), point_type(2,2));

        tester(s, "multilinestring((2 1,1 2),(4 0,4 10))",
               0, 0, strategy);
    }
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

    test_distance_of_geometries<linestring_type,multi_linestring_type> tester;

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

    test_distance_of_geometries<multi_linestring_type,multi_linestring_type>
        tester;

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
    test_distance_of_segment_and_geometry<polygon_type> tester;

    {
        segment_type s(point_type(-1,20), point_type(1,20));

        tester(s, "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
               10, 100, strategy);
    }
    {
        segment_type s(point_type(1,20), point_type(2,40));

        tester(s, "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
               10, 100, strategy);
  }
  {
      segment_type s(point_type(-1,20), point_type(-1,5));

      tester(s, "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
             0, 0, strategy);
  }
  {
      segment_type s(point_type(-1,20), point_type(-1,-20));

      tester(s, "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
             0, 0, strategy);
  }
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
    test_distance_of_geometries<linestring_type,polygon_type> tester;

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
    test_distance_of_geometries<polygon_type,polygon_type> tester;

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           "polygon((-5 20,5 20,5 25,-5 25,-5 20))",
           10, 100, strategy);

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           "polygon((-5 20,-5 5,5 5,5 20,-5 20))",
           0, 0, strategy);

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           "polygon((-5 20,-5 -20,5 -20,5 20,-5 20))",
           0, 0, strategy);

    tester("polygon((-10 -10,10 -10,10 10,-10 10,-10 -10),(-5 -5,-5 5,5 5,5 -5,-5 -5))",
           "polygon((-1 -1,0 0,-1 0,-1 -1))",
           4, 16, strategy);
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
    test_distance_of_segment_and_geometry<multi_polygon_type> tester;

    {
        segment_type s(point_type(-1,20), point_type(1,20));

        tester(s, "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                                ((0 22,-1 30, 2 40,0 22)))",
               2, 4, strategy);
    }
    {
        segment_type s(point_type(12,0), point_type(14,0));

        tester(s, "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                                ((20 -1,21 2,30 -10,20 -1)))",
               2, 4, strategy);
    }
    {
        segment_type s(point_type(12,0), point_type(20.5,0.5));
      
        tester(s, "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                                ((20 -1,21 2,30 -10,20 -1)))",
               0, 0, strategy);
    }
    {
        segment_type s(point_type(12,0), point_type(50,0));

        tester(s, "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                                ((20 -1,21 2,30 -10,20 -1)))",
               0, 0, strategy);
    }
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
    test_distance_of_geometries<linestring_type,multi_polygon_type> tester;

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
    test_distance_of_geometries<multi_linestring_type,multi_polygon_type>
        tester;

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
    test_distance_of_geometries<polygon_type,multi_polygon_type> tester;

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
    test_distance_of_geometries<multi_polygon_type,multi_polygon_type> tester;

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
void test_distance_point_box_2d(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "2D point/box distance tests" << std::endl;
#endif
    test_distance_of_box_and_geometry<point_type> tester;

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
void test_distance_point_box_3d(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "3D point/box distance tests" << std::endl;
#endif
    test_distance_of_box_and_geometry<point_type_3d> tester;

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

BOOST_AUTO_TEST_CASE( test_distance )
{
    point_segment_strategy strategy;

    test_distance_segment_segment(strategy);
    test_distance_segment_linestring(strategy);
    test_distance_linestring_linestring(strategy);
    test_distance_segment_multilinestring(strategy);
    test_distance_linestring_multilinestring(strategy);
    test_distance_multilinestring_multilinestring(strategy);

    test_distance_segment_polygon(strategy);
    test_distance_linestring_polygon(strategy);
    test_distance_polygon_polygon(strategy);
    test_distance_segment_multipolygon(strategy);
    test_distance_linestring_multipolygon(strategy);
    test_distance_multilinestring_multipolygon(strategy);
    test_distance_polygon_multipolygon(strategy);
    test_distance_multipolygon_multipolygon(strategy);

    test_distance_point_box_2d(point_point_strategy());
    test_distance_point_box_2d(strategy);

    test_distance_point_box_3d(point_point_strategy());
    test_distance_point_box_3d(strategy);

    test_more_empty_input<point_type,point_segment_strategy>(strategy);
}
