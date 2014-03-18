// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE test_distance_pointlike_areal
#endif

#include <boost/test/included/unit_test.hpp>

#include "test_distance_common.hpp"


typedef bg::model::point<double,2,bg::cs::cartesian>  point_type;
typedef bg::model::multi_point<point_type>            multi_point_type;
typedef bg::model::point<double,3,bg::cs::cartesian>  point_type_3d;
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


template <typename Strategy>
void test_distance_point_polygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "point/polygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<point_type, polygon_type> tester;

    tester("point(0 -20)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           10, 100, strategy);

    tester("point(12 0)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           2, 4, strategy);

    tester("point(0 0)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy);

    tester("point(0 0)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10),\
                    (-5 -5,-5 5,5 5,5 -5,-5 -5))",
           5, 25, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
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


template <typename Strategy>
void test_distance_multipoint_polygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multipoint/polygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_point_type, polygon_type> tester;

    tester("multipoint(0 -20,0 -15)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           5, 25, strategy);

    tester("multipoint(16 0,12 0)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           2, 4, strategy);

    tester("multipoint(0 0,5 5,4 4)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10))",
           0, 0, strategy);

    tester("multipoint(0 0,2 0)",
           "polygon((-10 -10,10 -10,10 10,-10 10,-10 -10),\
                    (-5 -5,-5 5,5 5,5 -5,-5 -5))",
           3, 9, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multipoint_multipolygon(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multipoint/multipolygon distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_point_type, multi_polygon_type> tester;

    tester("multipoint(0 -20,0 -15)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((0 22,-1 30,2 40,0 22)))",
           5, 25, strategy);

    tester("multipoint(16 0,12 0)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           2, 4, strategy);

    tester("multipoint(0 0,4 4,5 5)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10)),\
                         ((20 -1,21 2,30 -10,20 -1)))",
           0, 0, strategy);

    tester("multipoint(0 0,2 0)",
           "multipolygon(((-10 -10,10 -10,10 10,-10 10,-10 -10),\
                         (-5 -5,-5 5,5 5,5 -5,-5 -5)),\
                         ((100 0,101 0,101 1,100 1,100 0)))",
           3, 9, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
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

template <typename Strategy>
void test_distance_point_box_different_point_types(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "2D point/box distance tests with different points"
              << std::endl;
#endif
    typedef point_type double_point;
    typedef box_type double_box;
    typedef bg::model::point<int,2,bg::cs::cartesian> int_point;
    typedef bg::model::box<int_point> int_box;

    test_distance_of_geometries<int_point, int_box>()
        ("point(0 0)",
         make_box2d<int_box>(1, 1, 2, 2),
         sqrt(2), 2, strategy);

    test_distance_of_geometries<double_point, int_box>()
        ("point(0.5 0)",
         make_box2d<int_box>(1, -1, 2, 1),
         0.5, 0.25, strategy);

    test_distance_of_geometries<double_point, double_box>()
        ("point(1.5 0)",
         make_box2d<double_box>(1, -1, 2, 1),
         0, 0, strategy);

    test_distance_of_geometries<double_point, int_box>()
        ("point(1.5 0)",
         make_box2d<int_box>(1, -1, 2, 1),
         0, 0, strategy);

    test_distance_of_geometries<int_point, double_box>()
        ("point(1 0)",
         make_box2d<double_box>(0.5, -1, 1.5, 1),
         0, 0, strategy);

    test_distance_of_geometries<int_point, double_box>()
        ("point(0 0)",
         make_box2d<double_box>(0.5, -1, 1.5, 1),
         0.5, 0.25, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
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



template <typename Point, typename Strategy>
void test_more_empty_input_pointlike_areal(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "testing on empty inputs... " << std::flush;
#endif
    bg::model::multi_point<Point> multipoint_empty;
    bg::model::polygon<Point> polygon_empty;
    bg::model::multi_polygon<bg::model::polygon<Point> > multipolygon_empty;

    test_empty_input(multipoint_empty, polygon_empty, strategy);
    test_empty_input(multipoint_empty, multipolygon_empty, strategy);

#ifdef GEOMETRY_TEST_DEBUG
    std::cout << "done!" << std::endl;
#endif
}


//===========================================================================
//===========================================================================
//===========================================================================

BOOST_AUTO_TEST_CASE( test_all_point_polygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_polygon(pp_strategy);
    test_distance_point_polygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_multipolygon(pp_strategy);
    test_distance_point_multipolygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multipoint_polygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multipoint_polygon(pp_strategy);
    test_distance_multipoint_polygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multipoint_multipolygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multipoint_multipolygon(pp_strategy);
    test_distance_multipoint_multipolygon(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_box_2d )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_box_2d(pp_strategy);
    test_distance_point_box_2d(ps_strategy);

    test_distance_point_box_different_point_types(pp_strategy);
    test_distance_point_box_different_point_types(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_point_box_3d )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_point_box_3d(pp_strategy);
    test_distance_point_box_3d(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_empty_input_pointlike_areal )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_more_empty_input_pointlike_areal<point_type>(pp_strategy);
    test_more_empty_input_pointlike_areal<point_type>(ps_strategy);
}
