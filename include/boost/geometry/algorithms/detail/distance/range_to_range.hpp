// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_RANGE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_RANGE_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <boost/range.hpp>

#include <boost/geometry/core/is_areal.hpp>
#include <boost/geometry/core/point_type.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

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
        return range_to_range
            <
                Range2, Range1, Strategy,
                CollectPoints2, CollectPoints1,
                false, true
            >::apply(range2, range1, strategy, check_intersection);
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
        if ( boost::size(range1) == 1 )
        {
            return geometry::distance(*boost::begin(range1), range2, strategy);
        }

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
        if ( boost::size(range1) == 1 )
        {
            return geometry::distance(*boost::begin(range1), range2, strategy);
        }

        if ( boost::size(range2) == 1 )
        {
            return geometry::distance(*boost::begin(range2), range1, strategy);
        }

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


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

// linestring-linestring
template <typename Linestring1, typename Linestring2, typename Strategy>
struct distance
    <
        Linestring1, Linestring2, Strategy,
        linestring_tag, linestring_tag, 
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Linestring1, Linestring2, Strategy, false, false
        >
{};


// linestring-polygon
template <typename Linestring, typename Polygon, typename Strategy>
struct distance
    <
        Linestring, Polygon, Strategy,
        linestring_tag, polygon_tag, 
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Linestring, Polygon, Strategy, false, true
        >
{};


// linestring-ring
template <typename Linestring, typename Ring, typename Strategy>
struct distance
    <
        Linestring, Ring, Strategy,
        linestring_tag, ring_tag, 
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Linestring, Ring, Strategy, false, true
        >
{};


// polygon-polygon
template <typename Polygon1, typename Polygon2, typename Strategy>
struct distance
    <
        Polygon1, Polygon2, Strategy,
        polygon_tag, polygon_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Polygon1, Polygon2, Strategy, true, true
        >
{};


// polygon-ring
template <typename Polygon, typename Ring, typename Strategy>
struct distance
    <
        Polygon, Ring, Strategy,
        polygon_tag, ring_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Polygon, Ring, Strategy, true, true
        >
{};




} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH



}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_RANGE_TO_RANGE_HPP
