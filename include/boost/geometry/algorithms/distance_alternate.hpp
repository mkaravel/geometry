// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2014 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2014 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2014 Mateusz Loskot, London, UK.
// Copyright (c) 2013-2014 Adam Wulkiewicz, Lodz, Poland.

// This file was modified by Oracle on 2014.
// Modifications copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_ALGORITHMS_DISTANCE_ALTERNATE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DISTANCE_ALTERNATE_HPP

#ifdef BOOST_GEOMETRY_ALTERNATE_DISTANCE

#include <boost/geometry/algorithms/distance_basic.hpp>
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



#if 0
// compute range-polygon distance
template
<
    typename Range,
    typename Polygon,
    closure_selector RangeClosure,
    typename Strategy
>
struct range_to_polygon
{
    typedef typename return_type
        <
            Strategy,
            typename point_type<Range>::type,
            typename point_type<Polygon>::type
        >::type return_type;

    static inline return_type
    apply(Range const& range, Polygon const& polygon, Strategy const& strategy)
    {
        typedef typename geometry::ring_type<Polygon>::type e_ring;
        typedef typename geometry::interior_type<Polygon>::type i_rings;
        typedef typename range_value<i_rings>::type i_ring;

        if ( geometry::intersects(range, polygon) )
        {
            return 0;
        }

        e_ring const& ext_ring = geometry::exterior_ring<Polygon>(polygon);
        i_rings const& int_rings = geometry::interior_rings<Polygon>(polygon);

        return_type dmin = range_to_range
            <
                e_ring, Range, closure<Polygon>::value, RangeClosure, Strategy
            >::apply(ext_ring, range, strategy, false);

        typedef typename boost::range_iterator<i_rings const>::type iterator_type;
        for (iterator_type it = boost::begin(int_rings);
             it != boost::end(int_rings); ++it)
        {
            return_type d = range_to_range
                <
                    i_ring, Range,
                    closure<Polygon>::value, RangeClosure,
                    Strategy
                >::apply(*it, range, strategy, false);

            if ( d < dmin )
            {
                dmin = d;
            }
        }

        return dmin;
    }
};


// compute polygon-polygon distance
template <typename Polygon1, typename Polygon2, typename Strategy>
struct polygon_to_polygon
{
    typedef typename return_type
        <
            Strategy,
            typename point_type<Polygon1>::type,
            typename point_type<Polygon2>::type
        >::type return_type;

    static inline return_type
    apply(Polygon1 const& polygon1, Polygon2 const& polygon2,
          Strategy const& strategy)
    {
        // MK:: the following code can be possibly made more efficient by
        // finding and considering the rings which give the distance

        // check first if the two polygons intersect; in this case the
        // distance is 0
        if ( geometry::intersects(polygon1, polygon2) )
        {
            return 0;
        }

        // types for 1st polygon
        // =====================
        // types for polygon1 exterior ring
        typedef typename geometry::ring_type<Polygon1>::type e_ring1;

        // types for polygon1 interior rings
        typedef typename geometry::interior_type<Polygon1>::type i_rings1;
        typedef typename range_iterator<i_rings1 const>::type i_rings1_iterator;
        typedef typename range_value<i_rings1>::type i_ring1;

        // types for 2nd polygon
        // =====================
        // types for polygon2 exterior ring
        typedef typename geometry::ring_type<Polygon2>::type e_ring2;

        // types for polygon2 interior rings
        typedef typename geometry::interior_type<Polygon2>::type i_rings2;
        typedef typename range_iterator<i_rings2 const>::type i_rings2_iterator;
        typedef typename range_value<i_rings2>::type i_ring2;
    
        e_ring1 const& ext1_ring = geometry::exterior_ring<Polygon1>(polygon1);
        e_ring2 const& ext2_ring = geometry::exterior_ring<Polygon2>(polygon2);

        return_type dmin = range_to_range
            <
                e_ring1, e_ring2,
                closure<e_ring1>::value, closure<e_ring2>::value,
                Strategy
            >::apply(ext1_ring, ext2_ring, strategy, false);

        i_rings1 const& int_rings1 = geometry::interior_rings<Polygon1>(polygon1);
        i_rings2 const& int_rings2 = geometry::interior_rings<Polygon2>(polygon2);

        for (i_rings1_iterator it = boost::begin(int_rings1);
             it != boost::end(int_rings1); ++it)
        {
            return_type d = range_to_range
                <
                    i_ring1, e_ring2,
                    closure<i_ring1>::value, closure<e_ring2>::value,
                    Strategy
                >::apply(*it, ext2_ring, strategy, false);

            if ( d < dmin )
            {
                dmin = d;
            }
        }

        for (i_rings2_iterator it = boost::begin(int_rings2);
             it != boost::end(int_rings2); ++it)
        {
            return_type d = range_to_range
                <
                    e_ring1, i_ring2,
                    closure<e_ring1>::value, closure<i_ring2>::value,
                    Strategy
                >::apply(ext1_ring, *it, strategy, false);

            if ( d < dmin )
            {
                dmin = d;
            }
        }

        for (i_rings1_iterator it1 = boost::begin(int_rings1);
             it1 != boost::end(int_rings1); ++it1)
        {
            for (i_rings2_iterator it2 = boost::begin(int_rings2);
                 it2 != boost::end(int_rings2); ++it2)
            {
                return_type d = range_to_range
                    <
                        i_ring1, i_ring2,
                        closure<i_ring1>::value, closure<i_ring2>::value,
                        Strategy
                    >::apply(*it1, *it2, strategy, false);

                if ( d < dmin )
                {
                    dmin = d;
                }
            }
        }

        return dmin;
    }
};
#endif

}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


using strategy::distance::services::return_type;


// dispatch for linestring-linestring distances
template <typename Linestring1, typename Linestring2, typename Strategy>
struct distance
    <
        Linestring1, Linestring2, Strategy, linestring_tag, linestring_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Linestring1, Linestring2, Strategy, false, false
        >
{};

template <typename Linestring1, typename Linestring2, typename Strategy>
struct distance
    <
         Linestring1, Linestring2, Strategy, linestring_tag, linestring_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring1>::type,
            typename point_type<Linestring2>::type
        >::type
    apply(Linestring1 const& linestring1, Linestring2 const& linestring2,
          Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring1, Linestring2, Strategy
            >::type strategy_type;
        return detail::distance::range_to_range
            <
                Linestring1, Linestring2, strategy_type, false, false
            >::apply(linestring1, linestring2, strategy_type());
    }
};



// dispatch for linestring-ring distances
template <typename Linestring, typename Ring, typename Strategy>
struct distance
    <
        Linestring, Ring, Strategy, linestring_tag, ring_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Linestring, Ring, Strategy, false, false
        >
{};

template <typename Linestring, typename Ring, typename Strategy>
struct distance
    <
         Linestring, Ring, Strategy, linestring_tag, ring_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring>::type,
            typename point_type<Ring>::type
        >::type
    apply(Linestring const& linestring, Ring const& ring, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring, Ring, Strategy
            >::type strategy_type;
        return detail::distance::range_to_range
            <
                Linestring, Ring, strategy_type, false, false
            >::apply(linestring, ring, strategy_type());
    }
};



// dispatch for linestring-polygon distances
template <typename Linestring, typename Polygon, typename Strategy>
struct distance
    <
        Linestring, Polygon, Strategy, linestring_tag, polygon_tag,
        strategy_tag_distance_point_segment, false
    >
#if 1
    : detail::distance::range_to_range
        <
          Linestring, Polygon, Strategy, false, true
        >
#else
    : detail::distance::range_to_polygon
        <
            Linestring, Polygon, closure<Linestring>::value, Strategy
        >
#endif
{};

template<typename Linestring, typename Polygon, typename Strategy>
struct distance
    <
         Linestring, Polygon, Strategy, linestring_tag, polygon_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring>::type,
            typename point_type<Polygon>::type
        >::type
    apply(Linestring const& linestring, Polygon const& polygon, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring, Polygon, Strategy
            >::type strategy_type;
#if 1
        return detail::distance::range_to_range
            <
                Linestring, Polygon, strategy_type, false, true
            >::apply(linestring, polygon, strategy_type());
#else
        return detail::distance::range_to_polygon
            <
                Linestring, Polygon, closure<Linestring>::value, strategy_type
            >::apply(linestring, polygon, strategy_type());
#endif
    }
};


// dispatch for polygon-polygon distances
template <typename Polygon1, typename Polygon2, typename Strategy>
struct distance
    <
        Polygon1, Polygon2, Strategy, polygon_tag, polygon_tag,
        strategy_tag_distance_point_segment, false
    >
#if 1
    : detail::distance::range_to_range
        <
            Polygon1, Polygon2, Strategy, true, true
        >
#else
    : detail::distance::polygon_to_polygon<Polygon1, Polygon2, Strategy>
#endif
{};

template <typename Polygon1, typename Polygon2, typename Strategy>
struct distance
    <
         Polygon1, Polygon2, Strategy, polygon_tag, polygon_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Polygon1>::type,
            typename point_type<Polygon2>::type
        >::type
    apply(Polygon1 const& polygon1, Polygon2 const& polygon2, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Polygon1, Polygon2, Strategy
            >::type strategy_type;
#if 1
        return detail::distance::range_to_range
            <
                Polygon1, Polygon2, strategy_type, true, true
            >::apply(polygon1, polygon2, strategy_type());
#else
        return detail::distance::polygon_to_polygon
            <
                Polygon1, Polygon2, strategy_type
            >::apply(polygon1, polygon2, strategy_type());
#endif
    }
};



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALTERNATE_DISTANCE

#endif // BOOST_GEOMETRY_ALGORITHMS_DISTANCE_ALTERNATE_HPP
