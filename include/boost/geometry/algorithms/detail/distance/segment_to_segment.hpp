// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_SEGMENT_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_SEGMENT_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/util/calculation_type.hpp>

#include <boost/geometry/algorithms/detail/distance/default_strategies.hpp>
#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/algorithms/intersects.hpp>


namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{



// compute segment-segment distance
template<typename Segment1, typename Segment2, typename Strategy>
struct segment_to_segment
{
    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Segment1>::type,
            typename point_type<Segment2>::type
        >::type return_type;

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type comparable_strategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > get_comparable;

    static inline return_type
    apply(Segment1 const& segment1, Segment2 const& segment2,
          Strategy const& strategy)
    {
        if ( geometry::intersects(segment1, segment2) )
        {
            return 0;
        }

        typename point_type<Segment1>::type p[2];
        detail::assign_point_from_index<0>(segment1, p[0]);
        detail::assign_point_from_index<1>(segment1, p[1]);

        typename point_type<Segment2>::type q[2];
        detail::assign_point_from_index<0>(segment2, q[0]);
        detail::assign_point_from_index<1>(segment2, q[1]);

        return_type d[4];
        comparable_strategy cstrategy = get_comparable::apply(strategy);
        d[0] = cstrategy.apply(q[0], p[0], p[1]);
        d[1] = cstrategy.apply(q[1], p[0], p[1]);
        d[2] = cstrategy.apply(p[0], q[0], q[1]);
        d[3] = cstrategy.apply(p[1], q[0], q[1]);

        return_type dmin = d[0];
        for (std::size_t i = 1; i < 4; ++i)
        {
            if ( d[i] < dmin )
            {
                dmin = d[i];
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                comparable_strategy,
                Strategy,
                typename point_type<Segment1>::type,
                typename point_type<Segment2>::type
            >::apply(dmin);
    }
};




}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{



// segment-segment
template <typename Segment1, typename Segment2, typename Strategy>
struct distance
    <
        Segment1, Segment2, Strategy, segment_tag, segment_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::segment_to_segment<Segment1, Segment2, Strategy>
{};



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SEGMENT_TO_SEGMENT_HPP
