// Boost.Geometry

// Copyright (c) 2016, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_FORMULAS_GEOGRAPHIC_HPP
#define BOOST_GEOMETRY_FORMULAS_GEOGRAPHIC_HPP

#include <boost/geometry/core/coordinate_system.hpp>
#include <boost/geometry/core/coordinate_type.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/radian_access.hpp>

//#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>

#include <boost/geometry/formulas/eccentricity_sqr.hpp>
#include <boost/geometry/formulas/flattening.hpp>

#include <boost/geometry/util/math.hpp>
#include <boost/geometry/util/normalize_spheroidal_coordinates.hpp>
#include <boost/geometry/util/select_coordinate_type.hpp>

namespace boost { namespace geometry {
    
namespace formula {

template <typename Point3d, typename PointGeo, typename Spheroid>
inline Point3d geo_to_cart3d(PointGeo const& point_geo, Spheroid const& spheroid)
{
    typedef typename coordinate_type<Point3d>::type calc_t;

    calc_t const c1 = 1;
    calc_t const e_sqr = eccentricity_sqr<calc_t>(spheroid);

    calc_t const lon = get_as_radian<0>(point_geo);
    calc_t const lat = get_as_radian<1>(point_geo);

    Point3d res;

    calc_t const sin_lat = sin(lat);

    // "unit" spheroid, a = 1
    calc_t const N = c1 / math::sqrt(c1 - e_sqr * math::sqr(sin_lat));
    calc_t const N_cos_lat = N * cos(lat);

    set<0>(res, N_cos_lat * cos(lon));
    set<1>(res, N_cos_lat * sin(lon));
    set<2>(res, N * (c1 - e_sqr) * sin_lat);

    return res;
}

template <typename PointGeo, typename Spheroid, typename Point3d>
inline void geo_to_cart3d(PointGeo const& point_geo, Point3d & result, Point3d & north, Point3d & east, Spheroid const& spheroid)
{
    typedef typename coordinate_type<Point3d>::type calc_t;

    calc_t const c1 = 1;
    calc_t const e_sqr = eccentricity_sqr<calc_t>(spheroid);

    calc_t const lon = get_as_radian<0>(point_geo);
    calc_t const lat = get_as_radian<1>(point_geo);

    calc_t const sin_lon = sin(lon);
    calc_t const cos_lon = cos(lon);
    calc_t const sin_lat = sin(lat);
    calc_t const cos_lat = cos(lat);

    // "unit" spheroid, a = 1
    calc_t const N = c1 / math::sqrt(c1 - e_sqr * math::sqr(sin_lat));
    calc_t const N_cos_lat = N * cos_lat;

    set<0>(result, N_cos_lat * cos_lon);
    set<1>(result, N_cos_lat * sin_lon);
    set<2>(result, N * (c1 - e_sqr) * sin_lat);

    set<0>(east, -sin_lon);
    set<1>(east, cos_lon);
    set<2>(east, 0);

    set<0>(north, -sin_lat * cos_lon);
    set<1>(north, -sin_lat * sin_lon);
    set<2>(north, cos_lat);
}

template <typename PointGeo, typename Point3d, typename Spheroid>
inline PointGeo cart3d_to_geo(Point3d const& point_3d, Spheroid const& spheroid)
{
    typedef typename coordinate_type<PointGeo>::type coord_t;
    typedef typename coordinate_type<Point3d>::type calc_t;

    calc_t const c1 = 1;
    //calc_t const c2 = 2;
    calc_t const e_sqr = eccentricity_sqr<calc_t>(spheroid);

    calc_t const x = get<0>(point_3d);
    calc_t const y = get<1>(point_3d);
    calc_t const z = get<2>(point_3d);
    calc_t const xy_l = math::sqrt(math::sqr(x) + math::sqr(y));

    calc_t const lonr = atan2(y, x);
    
    // NOTE: Alternative version
    // http://www.iag-aig.org/attach/989c8e501d9c5b5e2736955baf2632f5/V60N2_5FT.pdf
    // calc_t const lonr = c2 * atan2(y, x + xy_l);
    
    calc_t const latr = atan2(z, (c1 - e_sqr) * xy_l);

    // NOTE: If h is equal to 0 then there is no need to improve value of latitude
    //       because then N_i / (N_i + h_i) = 1
    // http://www.navipedia.net/index.php/Ellipsoidal_and_Cartesian_Coordinates_Conversion

    PointGeo res;

    set_from_radian<0>(res, lonr);
    set_from_radian<1>(res, latr);

    coord_t lon = get<0>(res);
    coord_t lat = get<1>(res);

    math::normalize_spheroidal_coordinates
        <
            typename coordinate_system<PointGeo>::type::units,
            coord_t
        >(lon, lat);

    set<0>(res, lon);
    set<1>(res, lat);

    return res;
}

template <typename Point3d, typename Spheroid>
inline Point3d projected_to_xy(Point3d const& point_3d, Spheroid const& spheroid)
{
    typedef typename coordinate_type<Point3d>::type coord_t;    
    
    // len_xy = sqrt(x^2 + y^2)
    // r = len_xy - |z / tan(lat)|
    // assuming h = 0
    // lat = atan2(z, (1 - e^2) * len_xy);
    // |z / tan(lat)| = (1 - e^2) * len_xy
    // r = e^2 * len_xy
    // x_res = r * cos(lon) = e^2 * len_xy * x / len_xy = e^2 * x
    // y_res = r * sin(lon) = e^2 * len_xy * y / len_xy = e^2 * y
    
    coord_t const c0 = 0;
    coord_t const e_sqr = formula::eccentricity_sqr<coord_t>(spheroid);

    Point3d res;

    set<0>(res, e_sqr * get<0>(point_3d));
    set<1>(res, e_sqr * get<1>(point_3d));
    set<2>(res, c0);

    return res;
}

template <typename Point3d, typename Spheroid>
inline Point3d projected_to_surface(Point3d const& direction, Spheroid const& spheroid)
{
    typedef typename coordinate_type<Point3d>::type coord_t;

    coord_t const c0 = 0;
    coord_t const c2 = 2;
    coord_t const c4 = 4;

    // calculate the point of intersection of a ray and spheroid's surface
    // the origin is the origin of the coordinate system
    //(x*x+y*y)/(a*a) + z*z/(b*b) = 1
    // x = d.x * t
    // y = d.y * t
    // z = d.z * t        
    coord_t const dx = get<0>(direction);
    coord_t const dy = get<1>(direction);
    coord_t const dz = get<2>(direction);

    //coord_t const a_sqr = math::sqr(get_radius<0>(spheroid));
    //coord_t const b_sqr = math::sqr(get_radius<2>(spheroid));
    // "unit" spheroid, a = 1
    coord_t const a_sqr = 1;
    coord_t const b_sqr = math::sqr(get_radius<2>(spheroid) / get_radius<0>(spheroid));

    coord_t const param_a = (dx*dx + dy*dy) / a_sqr + dz*dz / b_sqr;

    coord_t const delta = c4 * param_a;
    coord_t const t = delta >= c0 ?
                      math::sqrt(delta) / (c2 * param_a) :
                      c0;

    // result = direction * t
    point_3d result = direction;
    multiply_value(result, t);

    return result;
}

template <typename Point3d, typename Spheroid>
inline Point3d projected_to_surface(Point3d const& origin, Point3d const& direction, Spheroid const& spheroid)
{
    typedef typename coordinate_type<Point3d>::type coord_t;

    coord_t const c0 = 0;
    coord_t const c1 = 1;
    coord_t const c2 = 2;
    coord_t const c4 = 4;

    // calculate the point of intersection of a ray and spheroid's surface
    //(x*x+y*y)/(a*a) + z*z/(b*b) = 1
    // x = o.x + d.x * t
    // y = o.y + d.y * t
    // z = o.z + d.z * t        
    coord_t const ox = get<0>(origin);
    coord_t const oy = get<1>(origin);
    coord_t const oz = get<2>(origin);
    coord_t const dx = get<0>(direction);
    coord_t const dy = get<1>(direction);
    coord_t const dz = get<2>(direction);

    //coord_t const a_sqr = math::sqr(get_radius<0>(spheroid));
    //coord_t const b_sqr = math::sqr(get_radius<2>(spheroid));
    // "unit" spheroid, a = 1
    coord_t const a_sqr = 1;
    coord_t const b_sqr = math::sqr(get_radius<2>(spheroid) / get_radius<0>(spheroid));

    coord_t const param_a = (dx*dx + dy*dy) / a_sqr + dz*dz / b_sqr;
    coord_t const param_b = c2 * ((ox*dx + oy*dy) / a_sqr + oz*dz / b_sqr);
    coord_t const param_c = (ox*ox + oy*oy) / a_sqr + oz*oz / b_sqr - c1;

    coord_t const delta = param_b*param_b - c4 * param_a*param_c;
    coord_t const t = delta >= c0 ?
                      (-param_b + math::sqrt(delta)) / (c2 * param_a) :
                      c0;

    // result = origin + direction * t
    point_3d result = direction;
    multiply_value(result, t);
    add_point(result, origin);

    return result;
}

} // namespace formula

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_FORMULAS_GEOGRAPHIC_HPP
