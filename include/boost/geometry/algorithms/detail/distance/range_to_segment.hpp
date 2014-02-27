// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_SEGMENT_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_SEGMENT_HPP

#include <boost/range.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/reverse_dispatch.hpp>
#include <boost/geometry/core/tag_cast.hpp>

#include <boost/geometry/geometries/segment.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <boost/geometry/views/closeable_view.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// compute range-segment distance
template
<
    typename Range,
    typename Segment,
    closure_selector Closure,
    typename Strategy
>
struct range_to_segment
{
    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Range>::type,
            typename point_type<Segment>::type
        >::type return_type;

#ifdef BOOST_GEOMETRY_USE_COMPARABLE_DISTANCES
    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    static inline return_type
    apply(Range const& range, Segment const& segment,
          Strategy const& strategy, bool check_intersection = true)
    {
        typedef typename strategy::distance::services::strategy_point_point
            <
                ComparableStrategy
            >::type pp_strategy_type;

        typedef point_to_range
            <
                typename point_type<Segment>::type,
                Range,
                Closure,
                pp_strategy_type,
                ComparableStrategy
            > segment_point_to_range;

        if ( check_intersection && geometry::intersects(range, segment) )
        {
            return 0;
        }

        // consider all distances from each endpoint of the segment
        // to the range, and then all distances of the points in the
        // range to the segment

        
        // initialize distance with one endpoint from the segment to
        // the range
        typename point_type<Segment>::type p[2];
        detail::assign_point_from_index<0>(segment, p[0]);
        detail::assign_point_from_index<1>(segment, p[1]);

        pp_strategy_type pp_strategy;
        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        return_type dmin = segment_point_to_range::apply(p[0],
                                                         range,
                                                         pp_strategy,
                                                         cstrategy);

        return_type d = segment_point_to_range::apply(p[1],
                                                      range,
                                                      pp_strategy,
                                                      cstrategy);

        if ( d < dmin )
        {
            dmin = d;
        }

        // check the distances from the points in the range to the segment
        typedef typename range_iterator<Range const>::type iterator_type;
        for (iterator_type it = boost::begin(range); it != boost::end(range); ++it)
        {
            d = cstrategy.apply(*it, p[0], p[1]);

            if ( d < dmin )
            {
                dmin = d;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                typename point_type<Range>::type,
                typename point_type<Segment>::type
            >::apply(dmin);
    }
#else
    static inline return_type
    apply(Range const& range, Segment const& segment,
          Strategy const& strategy, bool check_intersection = true)
    {
        typedef typename strategy::distance::services::strategy_point_point
            <
                Strategy
            >::type pp_strategy_type;

        typedef point_to_range
            <
                typename point_type<Segment>::type,
                Range,
                Closure,
                pp_strategy_type,
                Strategy
            > segment_point_to_range;

        if ( check_intersection &&
             geometry::intersects(range, segment) )
        {
            return 0;
        }

        // consider all distances from each endpoint of the segment
        // to the range, and then all distances of the points in the
        // range to the segment

        
        // initialize distance with one endpoint from the segment to
        // the range
        typename point_type<Segment>::type p[2];
        detail::assign_point_from_index<0>(segment, p[0]);
        detail::assign_point_from_index<1>(segment, p[1]);

        pp_strategy_type pp_strategy;

        return_type dmin = segment_point_to_range::apply(p[0],
                                                         range,
                                                         pp_strategy,
                                                         strategy);

        return_type d = segment_point_to_range::apply(p[1],
                                                      range,
                                                      pp_strategy,
                                                      strategy);

        if ( d < dmin )
        {
            dmin = d;
        }

        // check the distances from the points in the range to the segment
        typedef typename range_iterator<Range const>::type iterator_type;
        for (iterator_type it = boost::begin(range); it != boost::end(range); ++it)
        {
            d = strategy.apply(*it, p[0], p[1]);

            if ( d < dmin )
            {
                dmin = d;
            }
        }

        return dmin;
    }
#endif


    static inline return_type
    apply(Segment const& segment, Range const& range, 
          Strategy const& strategy, bool check_intersection = true)
    {
        return apply(range, segment, strategy, check_intersection);
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL




}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_SEGMENT_HPP
