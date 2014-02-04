// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_BOX_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_BOX_HPP

#include <boost/geometry/strategies/cartesian/distance_projected_point.hpp>
#include <boost/geometry/algorithms/detail/distance/point_to_box.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

namespace boost { namespace geometry
{

namespace detail { namespace distance
{


template
<
    typename Segment,
    typename Box,
    typename Strategy
>
struct distance_segment_box_2D
{
    typedef typename point_type<Segment>::type SegmentPoint;
    typedef typename point_type<Box>::type BoxPoint;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, SegmentPoint, BoxPoint
        >::type return_type;



    typedef typename strategy::distance::services::strategy_point_point
        <
            Strategy
        >::type pp_strategy_type;

    typedef point_to_box<SegmentPoint, Box, pp_strategy_type> PointToBox;




    static inline return_type apply(Segment const& segment,
                                    Box const& box,
                                    Strategy const& strategy)
    {
        if ( geometry::intersects(segment, box) )
        {
            return 0;
        }

        SegmentPoint p[2];
        detail::assign_point_from_index<0>(segment, p[0]);
        detail::assign_point_from_index<1>(segment, p[1]);

        return_type d[6];

        pp_strategy_type pp_strategy;
        d[0] = PointToBox::apply(p[0], box, pp_strategy);
        d[1] = PointToBox::apply(p[1], box, pp_strategy);

        d[2] = strategy.apply(box.min_corner(), p[0], p[1]);
        d[3] = strategy.apply(box.max_corner(), p[0], p[1]);

        BoxPoint top_left(geometry::get<0>(box.min_corner()),
                          geometry::get<1>(box.max_corner()));

        BoxPoint bottom_right(geometry::get<0>(box.max_corner()),
                              geometry::get<1>(box.min_corner()));

        d[4] = strategy.apply(top_left, p[0], p[1]);
        d[5] = strategy.apply(bottom_right, p[0], p[1]);

        return *std::min_element(d, d+6);
    }
};



template
<
    typename Segment,
    typename Box,
    std::size_t DimensionCount,
    typename Strategy
>
struct segment_to_box_dispatch
    : not_implemented<Segment, Box, Strategy>
{};


template
<
    typename Segment,
    typename Box,
    typename Strategy
>
struct segment_to_box_dispatch<Segment, Box, 2, Strategy>
    : distance_segment_box_2D<Segment, Box, Strategy>
{};


template <typename Segment, typename Box, typename Strategy>
struct segment_to_box
    : segment_to_box_dispatch
        <
            Segment, Box, dimension<Segment>::type::value, Strategy
        >
{};



}} // namespace detail::distance

}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_BOX_HPP
