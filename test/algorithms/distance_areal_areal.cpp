// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE test_distance_areal_areal
#endif

#include <boost/test/included/unit_test.hpp>

#include "test_distance_common.hpp"


typedef bg::model::point<double,2,bg::cs::cartesian>  point_type;
typedef bg::model::polygon<point_type, false>         polygon_type;
typedef bg::model::multi_polygon<polygon_type>        multi_polygon_type;
typedef bg::model::box<point_type>                    box_type;

namespace services = bg::strategy::distance::services;
typedef bg::default_distance_result<point_type>::type return_type;

typedef bg::strategy::distance::pythagoras<> point_point_strategy;
typedef bg::strategy::distance::projected_point<> point_segment_strategy;

//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
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


template <typename Strategy>
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


template <typename Strategy>
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


template <typename Strategy>
void test_distance_box_box(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "box/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<box_type, box_type> tester;

    tester(make_box2d<box_type>(5, 5, 10, 10),
           make_box2d<box_type>(0, 0, 1, 1),
           sqrt(32.0), 32, strategy);

    tester(make_box2d<box_type>(3, 8, 13, 18),
           make_box2d<box_type>(0, 0, 5, 5),
           3, 9, strategy);

    tester(make_box2d<box_type>(5, 5, 10, 10),
           make_box2d<box_type>(0, 0, 5, 5),
           0, 0, strategy);

    tester(make_box2d<box_type>(5, 5, 10, 10),
           make_box2d<box_type>(0, 0, 6, 6),
           0, 0, strategy);

    tester(make_box2d<box_type>(3, 5, 13, 15),
           make_box2d<box_type>(0, 0, 5, 5),
           0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_polygon_box(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "polygon/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<polygon_type, box_type> tester;

    tester("polygon((10 10,10 5,5 5,5 10,10 10))",
           make_box2d<box_type>(0, 0, 1, 1),
           sqrt(32.0), 32, strategy);

    tester("polygon((10 10,10 5,5 5,5 10,10 10))",
           make_box2d<box_type>(0, 0, 5, 5),
           0, 0, strategy);

    tester("polygon((10 10,10 5,5 5,5 10,10 10))",
           make_box2d<box_type>(0, 0, 6, 6),
           0, 0, strategy);

    tester("polygon((10 10,15 5,10 0,5 5,10 10))",
           make_box2d<box_type>(5, 0, 7.5, 2.5),
           0, 0, strategy);

    tester("polygon((10 10,15 5,10 0,5 5,10 10))",
           make_box2d<box_type>(5, 0, 6, 1),
           sqrt(4.5), 4.5, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Strategy>
void test_distance_multipolygon_box(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "multipolygon/box distance tests" << std::endl;
#endif
    test_distance_of_geometries<multi_polygon_type, box_type> tester;

    tester("multipolygon(((-10 -10,-10 -9,-9 -9,-9 -10,-10 -10)),\
                         ((2 2,2 3,3 3,3 2,2 2)))",
           make_box2d<box_type>(0, 0, 1, 1),
           sqrt(2.0), 2, strategy);

    tester("multipolygon(((-10 -10,-10 -9,-9 -9,-9 -10,-10 -10)),\
                         ((2 2,2 3,3 3,3 2,2 2)))",
           make_box2d<box_type>(0, 0, 2, 2),
           0, 0, strategy);

    tester("multipolygon(((-10 -10,-10 -9,-9 -9,-9 -10,-10 -10)),\
                         ((2 2,2 3,3 3,3 2,2 2)))",
           make_box2d<box_type>(0, 0, 2.5, 2),
           0, 0, strategy);
}


//===========================================================================
//===========================================================================
//===========================================================================


template <typename Point, typename Strategy>
void test_more_empty_input_areal_areal(Strategy const& strategy)
{
#ifdef GEOMETRY_TEST_DEBUG
    std::cout << std::endl;
    std::cout << "testing on empty inputs... " << std::flush;
#endif
    bg::model::polygon<Point> polygon_empty;
    bg::model::multi_polygon<bg::model::polygon<Point> > multipolygon_empty;

    test_empty_input(polygon_empty, polygon_empty, strategy);
    test_empty_input(polygon_empty, multipolygon_empty, strategy);
    test_empty_input(multipolygon_empty, multipolygon_empty, strategy);

#ifdef GEOMETRY_TEST_DEBUG
    std::cout << "done!" << std::endl;
#endif
}


//===========================================================================
//===========================================================================
//===========================================================================

BOOST_AUTO_TEST_CASE( test_all_polygon_polygon )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_polygon_polygon(pp_strategy);
    test_distance_polygon_polygon(ps_strategy);
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

BOOST_AUTO_TEST_CASE( test_all_box_box )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_box_box(pp_strategy);
    test_distance_box_box(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_polygon_box )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_polygon_box(pp_strategy);
    test_distance_polygon_box(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_multipolygon_box )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_distance_multipolygon_box(pp_strategy);
    test_distance_multipolygon_box(ps_strategy);
}

BOOST_AUTO_TEST_CASE( test_all_empty_input_areal_areal )
{
    point_point_strategy pp_strategy;
    point_segment_strategy ps_strategy;

    test_more_empty_input_areal_areal<point_type>(pp_strategy);
    test_more_empty_input_areal_areal<point_type>(ps_strategy);
}
