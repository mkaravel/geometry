// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_RANGE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_RANGE_HPP


#include <boost/geometry/algorithms/detail/distance/closest_distance_rtree.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// To avoid spurious namespaces here:
using strategy::distance::services::return_type;


// compute range-range distance
template
<
    typename Range1,
    typename Range2,
    typename Strategy,
    bool CollectPoints1,
    bool CollectPoints2,
    bool IsAreal1 = geometry::is_areal<Range1>::value,
    bool IsAreal2 = geometry::is_areal<Range2>::value
>
struct range_to_range
    : not_implemented<Range1, Range2>
{};


template
<
    typename Range1,
    typename Range2,
    typename Strategy,
    bool CollectPoints1,
    bool CollectPoints2
>
struct range_to_range
    <
        Range1, Range2, Strategy,
        CollectPoints1, CollectPoints2,
        true, true
    >
{
    // Algorithm:
    // compute the distance as the minimum over the distances between
    // 1. the vertices of the first linestring with the second linestring
    // 2. the vertices of the second linestring with the first linestring
    // this should make the code much simpler and I believe more efficient

    typedef typename return_type
        <
            Strategy,
            typename point_type<Range1>::type,
            typename point_type<Range2>::type
        >::type return_type;

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    static inline return_type
    apply(Range1 const& range1, Range2 const& range2,
          Strategy const& strategy, bool check_intersection = true)
    {
        if ( check_intersection && geometry::intersects(range1, range2) )
        {
            return 0;
        }

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        return_type dmin = detail::distance::linear_to_linear_rtree
            <
                Range1, Range2, ComparableStrategy,
                CollectPoints1, CollectPoints2
            >::apply(range1, range2, cstrategy, false);

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                typename point_type<Range1>::type,
                typename point_type<Range2>::type
            >::apply(dmin);
    }
};




template
<
    typename Range1,
    typename Range2,
    typename Strategy,
    bool CollectPoints1,
    bool CollectPoints2
>
struct range_to_range
    <
        Range1, Range2, Strategy,
        CollectPoints1, CollectPoints2,
        true, false
    >
    : range_to_range
        <
            Range2, Range1, Strategy,
            CollectPoints2, CollectPoints1,
            false, true
        >
{};


template
<
    typename Range1,
    typename Range2,
    typename Strategy,
    bool CollectPoints1,
    bool CollectPoints2
>
struct range_to_range
    <
        Range1, Range2, Strategy,
        CollectPoints1, CollectPoints2,
        false, true
    >
{
    typedef typename return_type
        <
            Strategy,
            typename point_type<Range1>::type,
            typename point_type<Range2>::type
        >::type return_type;

    static inline return_type
    apply(Range1 const& range1, Range2 const& range2,
          Strategy const& strategy, bool check_intersection = true)
    {
#if 0
        if ( boost::size(range1) == 1 )
        {
            return dispatch::distance
                <
                    typename point_type<Range1>::type,
                    Range2,
                    Strategy
                >::apply(*boost::begin(range1), range2, strategy);
        }
#endif
        return range_to_range
            <
                Range1, Range2, Strategy,
                CollectPoints1, CollectPoints2,
                true, true
            >::apply(range1, range2, strategy, check_intersection);
    }
};


template
<
    typename Range1,
    typename Range2,
    typename Strategy,
    bool CollectPoints1,
    bool CollectPoints2
>
struct range_to_range
    <
        Range1, Range2, Strategy,
        CollectPoints1, CollectPoints2,
        false, false
    >
{
    typedef typename return_type
        <
            Strategy,
            typename point_type<Range1>::type,
            typename point_type<Range2>::type
        >::type return_type;

    static inline return_type
    apply(Range1 const& range1, Range2 const& range2,
          Strategy const& strategy, bool check_intersection = true)
    {
#if 0
        if ( boost::size(range1) == 1 )
        {
            return dispatch::distance
                <
                    typename point_type<Range1>::type,
                    Range2,
                    Strategy
                >::apply(*boost::begin(range1), range2, strategy);
        }

        if ( boost::size(range2) == 1 )
        {
            return dispatch::distance
                <
                    typename point_type<Range2>::type,
                    Range1,
                    Strategy
                >::apply(*boost::begin(range2), range1, strategy);
        }
#endif
        return range_to_range
            <
                Range1, Range2, Strategy,
                CollectPoints1, CollectPoints2,
                true, true
            >::apply(range1, range2, strategy, check_intersection);
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_RANGE_HPP
