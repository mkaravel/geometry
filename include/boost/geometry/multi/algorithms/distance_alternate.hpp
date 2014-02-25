// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// This file was modified by Oracle on 2014.
// Modifications copyright (c) 2014, Oracle and/or its affiliates.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DISTANCE_ALTERNATE_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DISTANCE_ALTERNATE_HPP

#ifdef BOOST_GEOMETRY_ALTERNATE_DISTANCE

#include <boost/numeric/conversion/bounds.hpp>
#include <boost/range.hpp>

#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/core/geometry_id.hpp>
#include <boost/geometry/multi/core/point_type.hpp>
#include <boost/geometry/multi/geometries/concepts/check.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/multi/algorithms/num_points.hpp>
#include <boost/geometry/util/select_coordinate_type.hpp>

#include <boost/geometry/multi/algorithms/detail/distance/get_points.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/split_to_segments.hpp>

#include <boost/geometry/multi/multi.hpp>

namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{

template <typename MultiPoint1, typename MultiPoint2, typename Strategy>
struct multipoint_to_multipoint
{
    typedef typename point_type<MultiPoint1>::type Point1;
    typedef typename point_type<MultiPoint2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    static inline return_type apply(MultiPoint1 const& multipoint1,
                                    MultiPoint2 const& multipoint2,
                                    Strategy const& strategy)
    {
        if ( boost::size(multipoint1) > boost::size(multipoint2) )

        {
            return multipoint_to_multipoint
                <
                    MultiPoint2, MultiPoint1, Strategy
                >::apply(multipoint2, multipoint1, strategy);
        }

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        return_type min_cdist = range_to_range_rtree
            <
                MultiPoint1, MultiPoint2, ComparableStrategy
            >::apply(multipoint1, multipoint2, cstrategy);

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                MultiPoint1,
                MultiPoint2
            >::apply(min_cdist);
    }
};


template <typename MultiPoint, typename Geometry, typename Strategy>
struct multipoint_to_linear
{
    typedef typename point_type<MultiPoint>::type Point1;
    typedef typename point_type<Geometry>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    static inline return_type apply(MultiPoint const& multipoint,
                                    Geometry const& geometry,
                                    Strategy const& strategy)
    {
        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        return_type min_cdist = point_range_to_linear_rtree
            <
                MultiPoint, Geometry, ComparableStrategy
            >::apply(multipoint, geometry, cstrategy);

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                MultiPoint,
                Geometry
            >::apply(min_cdist);
    }

    static inline return_type apply(Geometry const& geometry,
                                    MultiPoint const& multipoint,
                                    Strategy const& strategy)
    {
        return apply(multipoint, geometry, strategy);
    }
};


#if 0
template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy,
    bool CollectPoints1,
    bool CollectPoints2
>
struct linear_to_linear
{
    typedef typename point_type<Geometry1>::type Point1;
    typedef typename point_type<Geometry2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2,
                                    Strategy const& strategy)
    {
        return_type min_cdist = linear_to_linear_rtree
            <
                Geometry1, Geometry2, Strategy,
                CollectPoints1, CollectPoints2
            >::apply(geometry1, geometry2, strategy);

        return strategy::distance::services::comparable_to_regular
            <
                typename strategy::distance::services::comparable_type
                    <
                        Strategy
                    >::type,
                Strategy,
                Geometry1,
                Geometry2
            >::apply(min_cdist);
    }
};
#endif


template<typename Geometry, typename MultiGeometry, typename Strategy>
struct distance_single_to_multi
    : private dispatch::distance
      <
          Geometry,
          typename range_value<MultiGeometry>::type,
          Strategy
      >
{
    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    typedef typename strategy::distance::services::return_type
                     <
                         Strategy,
                         typename point_type<Geometry>::type,
                         typename point_type<MultiGeometry>::type
                     >::type return_type;

    static inline return_type apply(Geometry const& geometry,
                MultiGeometry const& multi,
                Strategy const& strategy)
    {
        return_type min_cdist = return_type();
        bool first = true;

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        for (typename range_iterator<MultiGeometry const>::type it = boost::begin(multi);
                it != boost::end(multi);
                ++it, first = false)
        {
            return_type cdist = dispatch::distance
                <
                    Geometry,
                    typename range_value<MultiGeometry>::type,
                    ComparableStrategy
                >::apply(geometry, *it, cstrategy);

            if (first || cdist < min_cdist)
            {
                min_cdist = cdist;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Geometry,
                MultiGeometry
            >::apply(min_cdist);
    }
};

template<typename MultiGeometry, typename Geometry, typename Strategy>
struct distance_multi_to_single
{
    typedef
    typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<MultiGeometry>::type,
            typename point_type<Geometry>::type
        >::type  return_type;

    static inline return_type apply(MultiGeometry const& multi,
                                    Geometry const& geometry,
                                    Strategy const& strategy)
    {
        return distance_single_to_multi
            <
                Geometry, MultiGeometry, Strategy
            >::apply(geometry, multi, strategy);
    }
};

template<typename Multi1, typename Multi2, typename Strategy>
struct distance_multi_to_multi
    : private distance_single_to_multi
      <
          typename range_value<Multi1>::type,
          Multi2,
          Strategy
      >
{
    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    typedef typename strategy::distance::services::return_type
                     <
                         Strategy,
                         typename point_type<Multi1>::type,
                         typename point_type<Multi2>::type
                     >::type return_type;

    static inline return_type apply(Multi1 const& multi1,
                Multi2 const& multi2, Strategy const& strategy)
    {
        return_type min_cdist = return_type();
        bool first = true;

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        for(typename range_iterator<Multi1 const>::type it = boost::begin(multi1);
                it != boost::end(multi1);
                ++it, first = false)
        {
            return_type cdist = distance_single_to_multi
                <
                    typename range_value<Multi1>::type,
                    Multi2,
                    ComparableStrategy
                >::apply(*it, multi2, cstrategy);
            if (first || cdist < min_cdist)
            {
                min_cdist = cdist;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Multi1,
                Multi2
            >::apply(min_cdist);
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{



#if 0
template
<
    typename G1,
    typename G2,
    typename Strategy,
    typename SingleGeometryTag
>
struct distance
<
    G1, G2, Strategy,
    SingleGeometryTag, multi_tag, strategy_tag_distance_point_point,
    false
>
    : detail::distance::distance_single_to_multi<G1, G2, Strategy>
{};

template
<
    typename G1,
    typename G2,
    typename Strategy,
    typename SingleGeometryTag
>
struct distance
<
    G1, G2, Strategy,
    SingleGeometryTag, multi_tag,
    strategy_tag_distance_point_segment,
    false
>
    : detail::distance::distance_single_to_multi<G1, G2, Strategy>
{};



template <typename G1, typename G2, typename Strategy>
struct distance
<
    G1, G2, Strategy,
    multi_tag, multi_tag, strategy_tag_distance_point_point,
    false
>
    : detail::distance::distance_multi_to_multi<G1, G2, Strategy>
{};

template <typename G1, typename G2, typename Strategy>
struct distance
<
    G1, G2, Strategy,
    multi_tag, multi_tag,
    strategy_tag_distance_point_segment,
    false
>
    : detail::distance::distance_multi_to_multi<G1, G2, Strategy>
{};
#endif

// dispatch for multi-points
template
<
    typename Point, typename MultiPoint, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Point, MultiPoint, Strategy,
        point_tag, multi_point_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Point, MultiPoint, Strategy
        >
{};

template
<
    typename Linestring, typename MultiPoint, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Linestring, MultiPoint, Strategy,
        linestring_tag, multi_point_tag, StrategyTag, false
    > : detail::distance::multipoint_to_linear
        <
            MultiPoint, Linestring, Strategy
        >
{};

template
<
    typename MultiPoint1, typename MultiPoint2, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPoint1, MultiPoint2, Strategy,
        multi_point_tag, multi_point_tag, StrategyTag, false
    > : detail::distance::multipoint_to_multipoint
        <
            MultiPoint1, MultiPoint2, Strategy
        >
{};

template
<
    typename MultiPoint, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPoint, MultiLinestring, Strategy,
        multi_point_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::multipoint_to_linear
        <
            MultiPoint, MultiLinestring, Strategy
        >
{};

template
<
    typename MultiPoint, typename Segment, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPoint, Segment, Strategy,
        multi_point_tag, segment_tag, StrategyTag, false
    > : detail::distance::distance_multi_to_single<MultiPoint, Segment, Strategy>
{};


// dispatches for multilinestrings
template
<
    typename Point, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Point, MultiLinestring, Strategy,
        point_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Point, MultiLinestring, Strategy
        >
{};

template
<
    typename Linestring, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Linestring, MultiLinestring, Strategy,
        linestring_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Linestring, MultiLinestring, Strategy, false, true
        >
{};

template
<
    typename Ring, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Ring, MultiLinestring, Strategy,
        ring_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Ring, MultiLinestring, Strategy, false, true
        >
{};

template
<
    typename Polygon, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Polygon, MultiLinestring, Strategy,
        polygon_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Polygon, MultiLinestring, Strategy, true, true
        >
{};

template
<
    typename MultiLinestring1, typename MultiLinestring2, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiLinestring1, MultiLinestring2, Strategy,
        multi_linestring_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiLinestring1, MultiLinestring2, Strategy, true, true
        >
{};

template
<
    typename MultiLinestring, typename Segment, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiLinestring, Segment, Strategy,
        multi_linestring_tag, segment_tag, StrategyTag, false
    > : detail::distance::distance_multi_to_single
        <
            MultiLinestring, Segment, Strategy
        >
{};

// dispatches for multipolygons
template
<
    typename Point, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Point, MultiPolygon, Strategy,
        point_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Point, MultiPolygon, Strategy
        >
{};

template
<
    typename Linestring, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Linestring, MultiPolygon, Strategy,
        linestring_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Linestring, MultiPolygon, Strategy, false, true
        >
{};

template
<
    typename Ring, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Ring, MultiPolygon, Strategy,
        ring_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Ring, MultiPolygon, Strategy, false, true
        >
{};

template
<
    typename Polygon, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Polygon, MultiPolygon, Strategy,
        polygon_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Polygon, MultiPolygon, Strategy, true, true
        >
{};

template
<
    typename MultiLinestring, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiLinestring, MultiPolygon, Strategy,
        multi_linestring_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiLinestring, MultiPolygon, Strategy, true, true
        >
{};

template
<
    typename MultiPolygon1, typename MultiPolygon2, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPolygon1, MultiPolygon2, Strategy,
        multi_polygon_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiPolygon1, MultiPolygon2, Strategy, true, true
        >
{};

template
<
    typename MultiPolygon, typename Segment, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPolygon, Segment, Strategy,
        multi_polygon_tag, segment_tag, StrategyTag, false
    > : detail::distance::distance_multi_to_single
        <
            MultiPolygon, Segment, Strategy
        >
{};


} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALTERNATE_DISTANCE

#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DISTANCE_ALTERNATE_HPP
