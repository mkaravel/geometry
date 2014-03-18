// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_BOX_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_BOX_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/algorithms/detail/distance/point_to_box.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


template
<
    typename Segment,
    typename Box,
    typename Strategy
>
class distance_segment_box_2D
{
private:
    typedef typename point_type<Segment>::type SegmentPoint;
    typedef typename point_type<Box>::type BoxPoint;

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    typedef typename strategy::distance::services::strategy_point_point
        <
            ComparableStrategy
        >::type pp_strategy_type;

    typedef point_to_box<SegmentPoint, Box, pp_strategy_type> PointToBox;


public:
    typedef typename strategy::distance::services::return_type
        <
            ComparableStrategy, SegmentPoint, BoxPoint
        >::type return_type;


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

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        d[2] = cstrategy.apply(box.min_corner(), p[0], p[1]);
        d[3] = cstrategy.apply(box.max_corner(), p[0], p[1]);

        BoxPoint top_left, bottom_right;

        geometry::assign_values(top_left,
                                geometry::get<0>(box.min_corner()),
                                geometry::get<1>(box.max_corner()));

        geometry::assign_values(bottom_right,
                                geometry::get<0>(box.max_corner()),
                                geometry::get<1>(box.min_corner()));

        d[4] = cstrategy.apply(top_left, p[0], p[1]);
        d[5] = cstrategy.apply(bottom_right, p[0], p[1]);

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Segment,
                Box
            >::apply( *std::min_element(d, d+6) );
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

namespace splitted_dispatch
{


template
<
    typename Segment,
    typename Box,
    std::size_t SegmentDimension,
    std::size_t BoxDimension,
    typename Strategy
>
struct distance_segment_to_box
    : not_implemented<Segment, Box, Strategy>
{};


template
<
    typename Segment,
    typename Box,
    typename Strategy
>
struct distance_segment_to_box<Segment, Box, 2, 2, Strategy>
    : detail::distance::distance_segment_box_2D<Segment, Box, Strategy>
{};


} // namespace splitted_dispatch


// segment-box
template <typename Segment, typename Box, typename Strategy>
struct distance
    <
        Segment, Box, Strategy, segment_tag, box_tag,
        strategy_tag_distance_point_segment, false
    >
    : splitted_dispatch::distance_segment_to_box
        <
            Segment,
            Box,
            dimension<Segment>::type::value,
            dimension<Box>::type::value,
            Strategy
        >
{};



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_BOX_HPP
