// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_BOX_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_BOX_HPP

#include <boost/geometry/geometry.hpp>


namespace boost { namespace geometry
{

namespace detail { namespace distance
{


template
<
    typename Point,
    typename Box,
    typename ClosestPoint,
    std::size_t Dimension,
    std::size_t DimensionCount
>
struct closest_point_to_box
{
    typedef typename coordinate_type<Point>::type PointCoordinate;
    typedef typename coordinate_type<Box>::type BoxCoordinate;

    static inline void apply(Point const& point,
                             Box const& box,
                             ClosestPoint& cp)
    {
        PointCoordinate p_coord = geometry::get<Dimension>(point);
        BoxCoordinate b_min_coord = geometry::get<Dimension>(box.min_corner());
        BoxCoordinate b_max_coord = geometry::get<Dimension>(box.max_corner()); 

        if ( p_coord < b_min_coord )
        {
            geometry::set<Dimension>(cp, b_min_coord);
        }
        else if ( p_coord > b_max_coord )
        {
            geometry::set<Dimension>(cp, b_max_coord);
        }
        else
        {
            geometry::set<Dimension>(cp, p_coord);
        }
        
        closest_point_to_box
            <
                Point, Box, ClosestPoint, Dimension + 1, DimensionCount
            >::apply(point, box, cp);
    }
};


template
<
    typename Point,
    typename Box,
    typename ClosestPoint,
    std::size_t DimensionCount
>
struct closest_point_to_box
    <
        Point, Box, ClosestPoint, DimensionCount, DimensionCount
    >
{
    static inline void apply(Point const&, Box const&, ClosestPoint&)
    {
    }
};





template <typename Point, typename Box, typename Strategy>
struct point_to_box
{
    typedef typename point_type<Box>::type BoxPoint;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point, BoxPoint
        >::type return_type;

    static inline return_type apply(Point const& point,
                                    Box const& box,
                                    Strategy const& strategy)
    {
        BoxPoint cp;

        closest_point_to_box
            <
                Point,
                Box,
                BoxPoint,
                0,
                dimension<Point>::type::value
            >::apply(point, box, cp);

        return strategy.apply(point, cp);
    }
};



}} // namespace detail::distance

}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_BOX_HPP
