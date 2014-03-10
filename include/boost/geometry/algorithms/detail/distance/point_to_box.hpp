// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_BOX_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_BOX_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/util/calculation_type.hpp>

#include <boost/geometry/algorithms/detail/distance/default_strategies.hpp>


namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
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
class point_to_box
{
private:
    typedef typename util::calculation_type::geometric::binary
        <
            Point,
            typename point_type<Box>::type,
            void
        >::type ClosestPointCoordinateType;

    typedef typename boost::mpl::if_
        <
            boost::is_same
                <
                    typename coordinate_type<Point>::type,
                    ClosestPointCoordinateType
                >,
            Point,
            typename point_type<Box>::type
        >::type ClosestPoint;

public:
    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point, ClosestPoint
        >::type return_type;

    static inline return_type apply(Point const& point,
                                    Box const& box,
                                    Strategy const& strategy)
    {
        ClosestPoint cp;

        closest_point_to_box
            <
                Point,
                Box,
                ClosestPoint,
                0,
                dimension<Point>::value
            >::apply(point, box, cp);

        return strategy.apply(point, cp);
    }
};



}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

// point-box
template <typename Point, typename Box, typename Strategy>
struct distance
    <
         Point, Box, Strategy, point_tag, box_tag,
         strategy_tag_distance_point_point, false
    >
    : detail::distance::point_to_box<Point, Box, Strategy>
{};


template <typename Point, typename Box, typename Strategy>
struct distance
    <
         Point, Box, Strategy, point_tag, box_tag,
         strategy_tag_distance_point_segment, false
    >
{
    typedef typename strategy::distance::services::strategy_point_point
        <
            Strategy
        >::type pp_strategy_type;

    static inline typename return_type
        <
            pp_strategy_type,
            Point,
            typename point_type<Box>::type
        >::type
    apply(Point const& point, Box const& box, Strategy const&)
    {
        return detail::distance::point_to_box
            <
                Point, Box, pp_strategy_type
            >::apply(point, box, pp_strategy_type());
    }
};


} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_BOX_HPP
