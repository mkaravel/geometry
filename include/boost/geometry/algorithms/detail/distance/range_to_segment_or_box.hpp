// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_SEGMENT_OR_BOX_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_SEGMENT_OR_BOX_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <boost/range.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/reverse_dispatch.hpp>
#include <boost/geometry/core/tag_cast.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/detail/distance/get_points.hpp>

#include <boost/geometry/views/closeable_view.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// compute range-segment or box distance
template
<
    typename Range,
    typename SegmentOrBox,
    closure_selector Closure,
    typename Strategy
>
class range_to_segment_or_box
{
private:
    typedef typename point_type<SegmentOrBox>::type SegmentOrBoxPoint;
    typedef typename point_type<Range>::type RangePoint;

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    typedef typename strategy::distance::services::tag
       <
           ComparableStrategy
       >::type ComparableStrategyTag;

    typedef dispatch::distance
        <
            SegmentOrBoxPoint, Range, ComparableStrategy,
            point_tag, typename tag<Range>::type,
            ComparableStrategyTag, false
        > point_to_range;

    typedef dispatch::distance
        <
            RangePoint, SegmentOrBox, ComparableStrategy,
            point_tag, typename tag<SegmentOrBox>::type,
            ComparableStrategyTag, false
        > point_to_segment_or_box;

public:
    typedef typename strategy::distance::services::return_type
        <
            Strategy, RangePoint, SegmentOrBoxPoint
        >::type return_type;

    static inline return_type
    apply(Range const& range, SegmentOrBox const& segment_or_box,
          Strategy const& strategy, bool check_intersection = true)
    {
        if ( check_intersection && geometry::intersects(range, segment_or_box) )
        {
            return 0;
        }

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        // get all points of the segment or the box
        std::vector<SegmentOrBoxPoint> segment_or_box_points;
        get_points
            <
                SegmentOrBox
            >::apply(segment_or_box, std::back_inserter(segment_or_box_points));

        // consider all distances from each endpoint of the segment or box
        // to the range
        BOOST_AUTO_TPL(it, segment_or_box_points.begin());
        return_type cd_min = point_to_range::apply(*it, range, cstrategy);

        for (++it; it != segment_or_box_points.end(); ++it)
        {
            return_type cd = point_to_range::apply(*it, range, cstrategy);
            if ( cd < cd_min )
            {
                cd_min = cd;
            }
        }

        // consider all distances of the points in the range to the
        // segment or box
        typedef typename range_iterator<Range const>::type iterator_type;
        for (iterator_type it = boost::begin(range); it != boost::end(range); ++it)
        {
            return_type cd =
                point_to_segment_or_box::apply(*it, segment_or_box, cstrategy);

            if ( cd < cd_min )
            {
                cd_min = cd;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy, Strategy,
                RangePoint, SegmentOrBoxPoint
            >::apply(cd_min);
    }

    static inline return_type
    apply(SegmentOrBox const& segment_or_box, Range const& range, 
          Strategy const& strategy, bool check_intersection = true)
    {
        return apply(range, segment_or_box, strategy, check_intersection);
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL



#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


// linestring-segment
template <typename Linestring, typename Segment, typename Strategy>
struct distance
    <
        Linestring, Segment, Strategy, linestring_tag, segment_tag,
        strategy_tag_distance_point_segment, false
    >
        : detail::distance::range_to_segment_or_box
            <
                Linestring, Segment, closed, Strategy
            >
{};



// segment-ring
template <typename Segment, typename Ring, typename Strategy>
struct distance
    <
        Segment, Ring, Strategy, segment_tag, ring_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_segment_or_box
        <
            Ring, Segment, closure<Ring>::value, Strategy
        >
{};



// linestring-box
template <typename Linestring, typename Box, typename Strategy>
struct distance
    <
        Linestring, Box, Strategy, linestring_tag, box_tag,
        strategy_tag_distance_point_segment, false
    >
        : detail::distance::range_to_segment_or_box
            <
                Linestring, Box, closed, Strategy
            >
{};



// ring-box
template <typename Ring, typename Box, typename Strategy>
struct distance
    <
        Ring, Box, Strategy, ring_tag, box_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_segment_or_box
        <
            Ring, Box, closure<Ring>::value, Strategy
        >
{};




} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH



}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_SEGMENT_OR_BOX_HPP
