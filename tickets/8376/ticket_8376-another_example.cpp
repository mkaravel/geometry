#include <iostream>
#include <string>
#include <sstream>

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
    multi_polygon_type mpoly1, mpoly2, result;
    bg::read_wkt("MULTIPOLYGON(((1920 1660,1920 1462,3720 1462,3720 3262,1920 3262,1920 1958,2218 2189),(3718 1561,3360 2233,3718 1957),(2818 2653,2218 2189,2818 3253,3360 2233)))", mpoly1);
    bg::read_wkt("MULTIPOLYGON(((1918 2155,1918 1957,2818 2653,3718 1957,3718 2154,2818 3055)))", mpoly2);

    std::string expected_diff = "MULTIPOLYGON(((2218 2189,1920 1660,1920 1462,3720 1462,3720 3262,1920 3262,1920 2157,2562 2799,2818 3253,3043 2829,3718 2154,3718 1957,3718 1561,3360 2233,3718 1957,3360 2234,3360 2233,2818 2653,2218 2189)))";

    bool is_valid1 = bg::is_valid(mpoly1);
    bool is_valid2 = bg::is_valid(mpoly2);

    if ( is_valid1 )
    {
        std::cout << "multi-polygon #1 is valid ..." << std::endl;
    }
    else
    {
        std::cout << "multi-polygon #1 is NOT valid ..." << std::endl;
        bg::correct(mpoly1);
        BOOST_ASSERT( bg::is_valid(mpoly1) );
    }

    if ( is_valid2 )
    {
        std::cout << "multi-polygon #2 is valid ..." << std::endl;
    }
    else
    {
        std::cout << "multi-polygon #2 is NOT valid ..." << std::endl;
        bg::correct(mpoly2);
        BOOST_ASSERT( bg::is_valid(mpoly2) );
    }

    bg::difference(mpoly1, mpoly2, result);

    std::stringstream sstr;
    sstr << bg::wkt(result);

    BOOST_ASSERT( sstr.str() == expected_diff );

    std::cout << "expected diff: " << expected_diff << std::endl;
    std::cout << "computed diff: " << bg::wkt(result) << std::endl;
    std::cout << bg::dsv(result) << std::endl;
    std::cout << "# of points: " << bg::num_points(result) << std::endl;

    return 0;
}
