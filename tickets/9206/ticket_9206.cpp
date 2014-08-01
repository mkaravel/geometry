#include <iostream>

#include <boost/assert.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/multi/geometries/multi_geometries.hpp>

namespace bg = boost::geometry;

typedef bg::model::point<int, 2, bg::cs::cartesian> point_type;
typedef bg::model::polygon<point_type, false, false> polygon_type; //ccw, open
typedef bg::model::multi_polygon<polygon_type> multi_polygon_type; //ccw, open

int main()
{
    polygon_type p1, p2;
    multi_polygon_type result;
    bg::read_wkt("POLYGON((1374 1092,1734 1092,3174 2526,3174 2886,1374 2886))", p1);
    bg::read_wkt("POLYGON((2817 2449,2817 2603,3134 2603,3134 2449,2817 2449))", p2);

    bool is_valid1 = bg::is_valid(p1);
    bool is_valid2 = bg::is_valid(p2);

    if ( is_valid1 )
    {
        std::cout << "1st polygon is valid ..." << std::endl;
    }
    else
    {
        std::cout << "1st polygon is NOT valid ..." << std::endl;
        bg::correct(p1);
        BOOST_ASSERT( bg::is_valid(p1) );
    }

    if ( is_valid2 )
    {
        std::cout << "1st polygon is valid ..." << std::endl;
    }
    else
    {
        std::cout << "2nd polygon is NOT valid ..." << std::endl;
        bg::correct(p2);
        BOOST_ASSERT( bg::is_valid(p2) );
    }

    bg::intersection(p1, p2, result);

    std::cout << bg::wkt(result) << std::endl;
    std::cout << bg::dsv(result) << std::endl;
    std::cout << "# of points: " << bg::num_points(result) << std::endl;

    return 0;
}
