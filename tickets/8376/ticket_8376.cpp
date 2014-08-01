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
    polygon_type poly;
    multi_polygon_type mpoly, result;
    bg::read_wkt("POLYGON((1920 1462,3720 1462,3720 3262,1920 3262))", poly);

    bg::read_wkt("MULTIPOLYGON(((1918 1957,1918 1657,2218 2189)),((3718 1957,3360 2233,3718 1561)),((3360 2233,2818 3253,2218 2189,2818 2653)))", mpoly);

    bool is_valid1 = bg::is_valid(poly);
    bool is_valid2 = bg::is_valid(mpoly);

    if ( is_valid1 )
    {
        std::cout << "polygon is valid ..." << std::endl;
    }
    else
    {
        std::cout << "polygon is NOT valid ..." << std::endl;
        bg::correct(poly);
        BOOST_ASSERT( bg::is_valid(poly) );
    }

    if ( is_valid2 )
    {
        std::cout << "multi-polygon is valid ..." << std::endl;
    }
    else
    {
        std::cout << "multi-polygon is NOT valid ..." << std::endl;
        bg::correct(mpoly);
        BOOST_ASSERT( bg::is_valid(mpoly) );
    }

    bg::difference(poly, mpoly, result);

    std::cout << bg::wkt(result) << std::endl;
    std::cout << bg::dsv(result) << std::endl;
    std::cout << "# of points: " << bg::num_points(result) << std::endl;

    return 0;
}
