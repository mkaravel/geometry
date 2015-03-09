// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>
#include <fstream>

#include <boost/geometry/geometry.hpp>
#include "partition_old.hpp"
#include "partition_new.hpp"
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>


#ifdef BOOST_GEOMETRY_USE_TIMER
#include <boost/timer/timer.hpp>
#endif

namespace bg = ::boost::geometry;

typedef bg::model::d2::point_xy<double> point_type;
typedef bg::model::multi_point<point_type> multi_point_type;
typedef bg::model::box<point_type> box_type;


struct expand_box
{
    template <typename Box, typename Geometry>
    static inline void apply(Box& total, Geometry const& geometry)
    {
        bg::expand(total, bg::return_envelope<Box>(geometry));
    }
};

struct overlaps_box
{
    template <typename Box, typename Geometry>
    static inline bool apply(Box const& box, Geometry const& geometry)
    {
#ifdef BOOST_GEOMETRY_TEST_DEBUG
        std::cout << "### overlaps:" << std::endl;
        std::cout << "Geometry: " << bg::wkt(geometry) << std::endl;
        std::cout << "Box: " << bg::wkt(box) << std::endl;
        std::cout << "result: "
                  << (! bg::disjoint(geometry, box))
                  << std::endl;
#endif
        return ! bg::disjoint(geometry, box);
    }
};

class item_visitor_type
{
public:
    item_visitor_type()
        : m_stored_distance(0)
        , m_first(true)
    {}

    template <typename Item1, typename Item2>
    inline void apply(Item1 const& item1, Item2 const& item2)
    {
#ifdef BOOST_GEOMETRY_TEST_DEBUG
        std::cout << "*** visitor:" << std::endl;
        std::cout << "items: " << geometry::wkt(item1)
                  << " " << geometry::wkt(item2)
                  << std::endl;
        std::cout << "policy (intersects?): "
                  << Policy::apply(item1, item2)
                  << std::endl;
#endif
        double dist = bg::distance(item1, item2);
        if (m_first || dist < m_stored_distance)
        {
            m_first = false;
            m_stored_distance = dist;
        }
    }

    double computed_distance() const { return m_stored_distance; }

    void clear()
    {
        m_first = true;
        m_stored_distance = 0;
    }

private:
    double m_stored_distance;
    bool m_first;
};

// structs for partition -- end


int main(int argc, char** argv)
{
    multi_point_type mp1, mp2;

    if ( argc == 1 ) { return 0; }

    std::ifstream ifs(argv[1]);
    assert( ifs );

    std::size_t n_red, n_blue;

    ifs >> n_red >> n_blue;

    for (std::size_t i = 0; i < n_red; ++i)
    {
        double x, y;
        ifs >> x >> y;
        mp1.push_back( point_type(x, y) );
    }

    for (std::size_t i = 0; i < n_blue; ++i)
    {
        double x, y;
        ifs >> x >> y;
        mp2.push_back( point_type(x, y) );
    }

    std::cout.precision(16);
    std::cout << "MP1 size: " << boost::size(mp1) << std::endl;
    std::cout << "MP2 size: " << boost::size(mp2) << std::endl;

    // run old partition

    double computed_distance;
    item_visitor_type visitor;


    std::cout << "checking old partition (one collection)..." << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif
        visitor.clear();

        bg::partition_old
            <
                box_type,
                expand_box,
                overlaps_box
            >::apply(mp1, visitor);
    }
    std::cout << "computed distance = "
              << visitor.computed_distance() << std::endl;
    std::cout << std::endl << std::endl;


    std::cout << "checking new partition (one collection)..." << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif

        visitor.clear();

        bg::partition_new
            <
                box_type,
                expand_box,
                overlaps_box
            >::apply(mp1, visitor);
    }
    std::cout << "computed distance = "
              << visitor.computed_distance() << std::endl;
    std::cout << std::endl << std::endl;


    std::cout << "checking old partition (two collections)..." << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif

        visitor.clear();

        bg::partition_old
            <
                box_type,
                expand_box,
                overlaps_box
            >::apply(mp1, mp2, visitor);
    }
    std::cout << "computed distance = "
              << visitor.computed_distance() << std::endl;
    std::cout << std::endl << std::endl;


    std::cout << "checking new partition (two collections)..." << std::endl;
    {
#ifdef BOOST_GEOMETRY_USE_TIMER
        boost::timer::auto_cpu_timer t;
#endif

        visitor.clear();

        bg::partition_new
            <
                box_type,
                expand_box,
                overlaps_box
            >::apply(mp1, mp2, visitor);
    }
    std::cout << "computed distance = "
              << visitor.computed_distance() << std::endl;
    std::cout << std::endl << std::endl;


    return 0;
}
