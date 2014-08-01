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
    bg::read_wkt("POLYGON((813 277,1341 319,1863 361,2391 571,2913 361,3441 319,3963 277,3963 32767,813 32767))", p1);

    bg::read_wkt("POLYGON((813 277,1341 319,1863 571,2391 571,2913 571,3441 319,3963 277,3963 32767,813 32767))", p2);

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
        std::cout << "2nd polygon is valid ..." << std::endl;
    }
    else
    {
        std::cout << "2nd polygon is NOT valid ..." << std::endl;
        bg::correct(p2);
        BOOST_ASSERT( bg::is_valid(p2) );
    }

    bg::sym_difference(p1, p2, result);

    std::cout << bg::wkt(result) << std::endl;
    std::cout << bg::dsv(result) << std::endl;
    std::cout << "# of points: " << bg::num_points(result) << std::endl;

    return 0;
}
