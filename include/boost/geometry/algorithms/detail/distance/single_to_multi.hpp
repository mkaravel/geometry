// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2014 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2014 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2014 Mateusz Loskot, London, UK.

// This file was modified by Oracle on 2014.
// Modifications copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SINGLE_TO_MULTI_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SINGLE_TO_MULTI_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>
#include <boost/numeric/conversion/bounds.hpp>
#include <boost/range.hpp>

#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/core/geometry_id.hpp>
#include <boost/geometry/multi/core/point_type.hpp>
#include <boost/geometry/multi/geometries/concepts/check.hpp>

#include <boost/geometry/multi/algorithms/num_points.hpp>
#include <boost/geometry/util/select_coordinate_type.hpp>

// includes needed from multi.hpp -- start
#include <boost/geometry/multi/algorithms/covered_by.hpp>
#include <boost/geometry/multi/algorithms/disjoint.hpp>
#include <boost/geometry/multi/algorithms/for_each.hpp>
#include <boost/geometry/multi/algorithms/within.hpp>

#include <boost/geometry/multi/algorithms/detail/for_each_range.hpp>
#include <boost/geometry/multi/algorithms/detail/sections/range_by_section.hpp>
#include <boost/geometry/multi/algorithms/detail/sections/sectionalize.hpp>

#include <boost/geometry/multi/views/detail/range_type.hpp>
// includes needed from multi.hpp -- end

#include <boost/geometry/algorithms/detail/distance/range_to_range.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// generic single-to-multi
template
<
    typename Geometry,
    typename MultiGeometry,
    typename Strategy,
    typename GeometryTag,
    typename MultiGeometryTag
>
struct distance_single_to_multi_generic
//    : private dispatch::distance
//      <
//          Geometry,
//          typename range_value<MultiGeometry>::type,
//          Strategy
//      >
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



// generic multi-to-single
template
<
    typename MultiGeometry,
    typename Geometry,
    typename Strategy,
    typename MultiGeometryTag,
    typename GeometryTag
>
struct distance_multi_to_single_generic
{
    typedef typename strategy::distance::services::return_type
                     <
                         Strategy,
                         typename point_type<MultiGeometry>::type,
                         typename point_type<Geometry>::type
                     >::type return_type;

    static inline return_type apply(MultiGeometry const& multi,
                                    Geometry const& geometry,
                                    Strategy const& strategy)
    {
        return distance_single_to_multi_generic
            <
                Geometry, MultiGeometry, Strategy,
                GeometryTag, MultiGeometryTag
            >::apply(geometry, multi, strategy);
    }
};




}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL



#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


// default sub-dispatch for single-multi
template
<
    typename Geometry,
    typename MultiGeometry,
    typename Strategy,
    typename GeometryTag,
    typename MultiGeometryTag
>
struct distance_single_to_multi
    : detail::distance::distance_single_to_multi_generic
        <
            Geometry, MultiGeometry, Strategy,
            GeometryTag, typename tag<MultiGeometry>::type
        >
{};


// linestring-multilinestring
template<typename Linestring, typename MultiLinestring, typename Strategy>
struct distance_single_to_multi
    <
        Linestring, MultiLinestring, Strategy,
        linestring_tag, multi_linestring_tag
    > : detail::distance::range_to_range
        <
            Linestring, MultiLinestring, Strategy, false, true
        >
{};


// linestring-multipolygon
template <typename Linestring, typename MultiPolygon, typename Strategy>
struct distance_single_to_multi
    <
        Linestring, MultiPolygon, Strategy,
        linestring_tag, multi_polygon_tag
    > : detail::distance::range_to_range
        <
            Linestring, MultiPolygon, Strategy, false, true
        >
{};


// polygon-multilinestring
template <typename Polygon, typename MultiLinestring, typename Strategy>
struct distance_single_to_multi
    <
        Polygon, MultiLinestring, Strategy,
        polygon_tag, multi_linestring_tag
    > : detail::distance::range_to_range
        <
            Polygon, MultiLinestring, Strategy, true, false
        >
{};


// polygon-multipolygon
template <typename Polygon, typename MultiPolygon, typename Strategy>
struct distance_single_to_multi
    <
        Polygon, MultiPolygon, Strategy,
        polygon_tag, multi_polygon_tag
    > : detail::distance::range_to_range
        <
            Polygon, MultiPolygon, Strategy, true, true
        >
{};



// default sub-dispatch for multi-single
template
<
    typename MultiGeometry,
    typename Geometry,
    typename Strategy,
    typename MultiGeometryTag,
    typename GeometryTag
>
struct distance_multi_to_single
    : detail::distance::distance_multi_to_single_generic
        <
            MultiGeometry, Geometry, Strategy,
            typename tag<MultiGeometry>::type, GeometryTag
        >
{};


// multilinestring-ring
template <typename MultiLinestring, typename Ring, typename Strategy>
struct distance_single_to_multi
    <
        MultiLinestring, Ring, Strategy,
        multi_linestring_tag, ring_tag
    > : detail::distance::range_to_range
        <
            MultiLinestring, Ring, Strategy, false, true
        >
{};


// multipolygon-ring
template <typename MultiPolygon, typename Ring, typename Strategy>
struct distance_single_to_multi
    <
        MultiPolygon, Ring, Strategy,
        multi_polygon_tag, ring_tag
    > : detail::distance::range_to_range
        <
            MultiPolygon, Ring, Strategy, true, true
        >
{};



// dispatch for single-multi geometry combinations
template
<
    typename Geometry,
    typename MultiGeometry,
    typename Strategy,
    typename GeometryTag
>
struct distance
    <
        Geometry, MultiGeometry, Strategy, GeometryTag, multi_tag,
        strategy_tag_distance_point_segment, false
    > : distance_single_to_multi
        <
            Geometry, MultiGeometry, Strategy,
            GeometryTag, typename tag<MultiGeometry>::type
        >
{};



// generic dispatch for multi-single geometry combinations
template
<
    typename MultiGeometry,
    typename Geometry,
    typename Strategy,
    typename GeometryTag
>
struct distance
    <
        MultiGeometry, Geometry, Strategy, multi_tag, GeometryTag,
        strategy_tag_distance_point_segment, false
    > : distance_multi_to_single
        <
            MultiGeometry, Geometry, Strategy,
            typename tag<MultiGeometry>::type, GeometryTag
        >
{};



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SINGLE_TO_MULTI_HPP
