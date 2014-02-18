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

#ifndef BOOST_GEOMETRY_ALGORITHMS_DISTANCE_BASIC_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DISTANCE_BASIC_HPP

#ifdef BOOST_GEOMETRY_ALTERNATE_DISTANCE

#include <boost/concept_check.hpp>
#include <boost/mpl/if.hpp>
#include <boost/range.hpp>
#include <boost/typeof/typeof.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/reverse_dispatch.hpp>
#include <boost/geometry/core/tag_cast.hpp>

#include <boost/geometry/algorithms/not_implemented.hpp>
#include <boost/geometry/algorithms/detail/throw_on_empty_input.hpp>

#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/concepts/check.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/default_distance_result.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <boost/geometry/views/closeable_view.hpp>
#include <boost/geometry/util/math.hpp>

#include <boost/geometry/algorithms/detail/distance/point_to_box.hpp>
#include <boost/geometry/algorithms/detail/distance/segment_to_box.hpp>

#include <boost/geometry/strategies/cartesian/distance_comparable_to_regular.hpp>

namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// To avoid spurious namespaces here:
using strategy::distance::services::return_type;


// Helper metafunction for default point-segment strategy retrieval
template <typename Geometry1, typename Geometry2, typename Strategy>
struct default_ps_strategy
    : strategy::distance::services::default_strategy
          <
              segment_tag,
              typename point_type<Geometry1>::type,
              typename point_type<Geometry2>::type,
              typename cs_tag<typename point_type<Geometry1>::type>::type,
              typename cs_tag<typename point_type<Geometry2>::type>::type,
              Strategy
          >
{};


template <typename P1, typename P2, typename Strategy>
struct point_to_point
{
    static inline typename return_type<Strategy, P1, P2>::type
    apply(P1 const& p1, P2 const& p2, Strategy const& strategy)
    {
        boost::ignore_unused_variable_warning(strategy);
        return strategy.apply(p1, p2);
    }
};


template<typename Point, typename Segment, typename Strategy>
struct point_to_segment
{
    static inline typename return_type
        <
            Strategy,
            Point,
            typename point_type<Segment>::type
        >::type
    apply(Point const& point, Segment const& segment, Strategy const& )
    {
        typename detail::distance::default_ps_strategy
            <
                Point,
                typename point_type<Segment>::type,
                Strategy
            >::type segment_strategy;

        typename point_type<Segment>::type p[2];
        geometry::detail::assign_point_from_index<0>(segment, p[0]);
        geometry::detail::assign_point_from_index<1>(segment, p[1]);
        return segment_strategy.apply(point, p[0], p[1]);
    }
};


// compute segment-segment distance
template<typename Segment1, typename Segment2, typename Strategy>
struct segment_to_segment
{
    typedef typename return_type
        <
            Strategy,
            typename point_type<Segment1>::type,
            typename point_type<Segment2>::type
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

#ifdef BOOST_GEOMETRY_USE_COMPARABLE_DISTANCES
        return_type d[4];
        ComparableStrategy cstrategy = GetComparable::apply(strategy);
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
                ComparableStrategy,
                Strategy,
                typename point_type<Segment1>::type,
                typename point_type<Segment2>::type
            >::apply(dmin);
#else
        return_type d[4];
        d[0] = strategy.apply(q[0], p[0], p[1]);
        d[1] = strategy.apply(q[1], p[0], p[1]);
        d[2] = strategy.apply(p[0], q[0], q[1]);
        d[3] = strategy.apply(p[1], q[0], q[1]);

        return_type dmin = d[0];
        for (std::size_t i = 1; i < 4; ++i)
        {
            if ( d[i] < dmin )
            {
                dmin = d[i];
            }
        }

        return dmin;
#endif
    }
};


template
<
    typename Point,
    typename Range,
    closure_selector Closure,
    typename PPStrategy,
    typename PSStrategy
>
struct point_to_range
{
    typedef typename return_type<PSStrategy, Point, typename point_type<Range>::type>::type return_type;

    typedef typename strategy::distance::services::comparable_type
        <
            PPStrategy
        >::type ComparablePPStrategy;

    typedef typename strategy::distance::services::comparable_type
        <
            PSStrategy
        >::type ComparablePSStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            PPStrategy
        > GetComparablePP;

    typedef typename strategy::distance::services::get_comparable
        <
            PSStrategy
        > GetComparablePS;
 
    static inline return_type apply(Point const& point, Range const& range,
            PPStrategy const& pp_strategy, PSStrategy const& ps_strategy)
    {
        ComparablePPStrategy epp_strategy = GetComparablePP::apply(pp_strategy);
        ComparablePSStrategy eps_strategy = GetComparablePS::apply(ps_strategy);

        return_type const zero = return_type(0);

        if (boost::size(range) == 0)
        {
            return zero;
        }

        typedef typename closeable_view<Range const, Closure>::type view_type;

        view_type view(range);

        // line of one point: return point distance
        typedef typename boost::range_iterator<view_type const>::type iterator_type;
        iterator_type it = boost::begin(view);
        iterator_type prev = it++;
        if (it == boost::end(view))
        {
            return strategy::distance::services::comparable_to_regular
            <
                ComparablePPStrategy,
                PPStrategy,
                Point,
                typename point_type<Range>::type
            >::apply( epp_strategy.apply(point, *boost::begin(view)) );
        }

        // start with first segment distance
        return_type d = eps_strategy.apply(point, *prev, *it);

        // check if other segments are closer
        for (++prev, ++it; it != boost::end(view); ++prev, ++it)
        {
            return_type const ds = eps_strategy.apply(point, *prev, *it);
            if (geometry::math::equals(ds, zero))
            {
                return ds;
            }
            else if (ds < d)
            {
                d = ds;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparablePSStrategy,
                PSStrategy,
                Point,
                typename point_type<Range>::type
            >::apply(d);
    }
};


template
<
    typename Point,
    typename Ring,
    closure_selector Closure,
    typename PPStrategy,
    typename PSStrategy
>
struct point_to_ring
{
    typedef std::pair
        <
            typename return_type<PPStrategy, Point, typename point_type<Ring>::type>::type, bool
        > distance_containment;

    static inline distance_containment apply(Point const& point,
                Ring const& ring,
                PPStrategy const& pp_strategy, PSStrategy const& ps_strategy)
    {
        return distance_containment
            (
                point_to_range
                    <
                        Point,
                        Ring,
                        Closure,
                        PPStrategy,
                        PSStrategy
                    >::apply(point, ring, pp_strategy, ps_strategy),
                geometry::within(point, ring)
            );
    }
};



template
<
    typename Point,
    typename Polygon,
    closure_selector Closure,
    typename PPStrategy,
    typename PSStrategy
>
struct point_to_polygon
{
    typedef typename return_type<PPStrategy, Point, typename point_type<Polygon>::type>::type return_type;
    typedef std::pair<return_type, bool> distance_containment;

    typedef typename strategy::distance::services::comparable_type
        <
            PPStrategy
        >::type ComparablePPStrategy;

    typedef typename strategy::distance::services::comparable_type
        <
            PSStrategy
        >::type ComparablePSStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            PPStrategy
        > GetComparablePP;

    typedef typename strategy::distance::services::get_comparable
        <
            PSStrategy
        > GetComparablePS;

    static inline distance_containment apply(Point const& point,
                Polygon const& polygon,
                PPStrategy const& pp_strategy, PSStrategy const& ps_strategy)
    {
        ComparablePPStrategy cpp_strategy = GetComparablePP::apply(pp_strategy);
        ComparablePSStrategy cps_strategy = GetComparablePS::apply(ps_strategy);

        // Check distance to all rings
        typedef point_to_ring
            <
                Point,
                typename ring_type<Polygon>::type,
                Closure,
                ComparablePPStrategy,
                ComparablePSStrategy
            > per_ring;

        distance_containment dc = per_ring::apply(point,
                        exterior_ring(polygon), cpp_strategy, cps_strategy);

        typename interior_return_type<Polygon const>::type rings
                    = interior_rings(polygon);
        for (BOOST_AUTO_TPL(it, boost::begin(rings)); it != boost::end(rings); ++it)
        {
            distance_containment dcr = per_ring::apply(point,
                            *it, cpp_strategy, cps_strategy);
            if (dcr.first < dc.first)
            {
                dc.first = dcr.first;
            }
            // If it was inside, and also inside inner ring,
            // turn off the inside-flag, it is outside the polygon
            if (dc.second && dcr.second)
            {
                dc.second = false;
            }
        }

        return_type rd = strategy::distance::services::comparable_to_regular
            <
                ComparablePSStrategy,
                PSStrategy,
                Point,
                typename point_type<Polygon>::type
            >::apply(dc.first);

        return std::make_pair(rd, dc.second);
    }
};


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
    typedef typename return_type
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
};


// compute polygon-segment distance
template <typename Polygon, typename Segment, typename Strategy>
struct polygon_to_segment
{
    typedef typename return_type
        <
            Strategy,
            typename point_type<Polygon>::type,
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
    apply(Polygon const& polygon, Segment const& segment,
          Strategy const& strategy)
    {
        typedef typename geometry::ring_type<Polygon>::type e_ring;
        typedef typename geometry::interior_type<Polygon>::type i_rings;
        typedef typename range_value<i_rings>::type i_ring;

        if ( geometry::intersects(polygon, segment) )
        {
            return 0;
        }

        e_ring const& ext_ring = geometry::exterior_ring<Polygon>(polygon);
        i_rings const& int_rings = geometry::interior_rings<Polygon>(polygon);

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        return_type dmin = range_to_segment
            <
                e_ring, Segment, closure<Polygon>::value, ComparableStrategy
            >::apply(ext_ring, segment, cstrategy, false);

        typedef typename boost::range_iterator<i_rings const>::type iterator_type;
        for (iterator_type it = boost::begin(int_rings);
             it != boost::end(int_rings); ++it)
        {
            return_type d = range_to_segment
                <
                    i_ring, Segment, closure<Polygon>::value, ComparableStrategy
                >::apply(*it, segment, cstrategy, false);

            if ( d < dmin )
            {
                dmin = d;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                typename point_type<Polygon>::type,
                typename point_type<Segment>::type
            >::apply(dmin);
    }
#else
    static inline return_type
    apply(Polygon const& polygon, Segment const& segment,
          Strategy const& strategy)
    {
        typedef typename geometry::ring_type<Polygon>::type e_ring;
        typedef typename geometry::interior_type<Polygon>::type i_rings;
        typedef typename range_value<i_rings>::type i_ring;

        if ( geometry::intersects(polygon, segment) )
        {
            return 0;
        }

        e_ring const& ext_ring = geometry::exterior_ring<Polygon>(polygon);
        i_rings const& int_rings = geometry::interior_rings<Polygon>(polygon);

        return_type dmin = range_to_segment
            <
                e_ring, Segment, closure<Polygon>::value, Strategy
            >::apply(ext_ring, segment, strategy, false);

        typedef typename boost::range_iterator<i_rings const>::type iterator_type;
        for (iterator_type it = boost::begin(int_rings);
             it != boost::end(int_rings); ++it)
        {
            return_type d = range_to_segment
                <
                    i_ring, Segment, closure<Polygon>::value, Strategy
                >::apply(*it, segment, strategy, false);

            if ( d < dmin )
            {
                dmin = d;
            }
        }

        return dmin;
    }
#endif
};


// Helper metafunction for default strategy retrieval
template <typename Geometry1, typename Geometry2>
struct default_strategy
    : strategy::distance::services::default_strategy
          <
              point_tag,
              typename point_type<Geometry1>::type,
              typename point_type<Geometry2>::type
          >
{};



}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


using strategy::distance::services::return_type;

#if 1
template
<
    typename Geometry1, typename Geometry2,
    typename Strategy = typename detail::distance::default_strategy<Geometry1, Geometry2>::type,
    typename Tag1 = typename tag<Geometry1>::type,
    typename Tag2 = typename tag<Geometry2>::type,
    typename StrategyTag = typename strategy::distance::services::tag<Strategy>::type,
    bool Reverse = reverse_dispatch<Geometry1, Geometry2>::type::value
>
struct distance: not_implemented<Tag1, Tag2>
{};
#else
template
<
    typename Geometry1, typename Geometry2,
    typename Strategy = typename detail::distance::default_strategy<Geometry1, Geometry2>::type,
    typename Tag1 = typename tag_cast<typename tag<Geometry1>::type, multi_tag>::type,
    typename Tag2 = typename tag_cast<typename tag<Geometry2>::type, multi_tag>::type,
    typename StrategyTag = typename strategy::distance::services::tag<Strategy>::type,
    bool Reverse = reverse_dispatch<Geometry1, Geometry2>::type::value
>
struct distance: not_implemented<Tag1, Tag2>
{};
#endif

// If reversal is needed, perform it
template
<
    typename Geometry1, typename Geometry2, typename Strategy,
    typename Tag1, typename Tag2, typename StrategyTag
>
struct distance
<
    Geometry1, Geometry2, Strategy,
    Tag1, Tag2, StrategyTag,
    true
>
    : distance<Geometry2, Geometry1, Strategy, Tag2, Tag1, StrategyTag, false>
{
    typedef typename strategy::distance::services::return_type
                     <
                         Strategy,
                         typename point_type<Geometry2>::type,
                         typename point_type<Geometry1>::type
                     >::type return_type;

    static inline return_type apply(
        Geometry1 const& g1,
        Geometry2 const& g2,
        Strategy const& strategy)
    {
        return distance
            <
                Geometry2, Geometry1, Strategy,
                Tag2, Tag1, StrategyTag,
                false
            >::apply(g2, g1, strategy);
    }
};


// Point-point
template <typename P1, typename P2, typename Strategy>
struct distance
    <
        P1, P2, Strategy,
        point_tag, point_tag, strategy_tag_distance_point_point,
        false
    >
    : detail::distance::point_to_point<P1, P2, Strategy>
{};


// Point-point with the point-segment strategy passed
template <typename P1, typename P2, typename Strategy>
struct distance
    <
        P1, P2, Strategy,
        point_tag, point_tag, strategy_tag_distance_point_segment,
        false
    >
{
    typedef typename return_type<Strategy, P1, P2>::type return_type;

    static inline return_type apply(P1 const& p1, P2 const& p2, Strategy&)
    {
        typedef typename strategy::distance::services::strategy_point_point
            <
                Strategy
            >::type pp_strategy;

        return detail::distance::point_to_point
            <
                P1, P2, pp_strategy
            >::apply(p1, p2, pp_strategy());
    }
};


// Point-line version 1, where point-point strategy is specified
template <typename Point, typename Linestring, typename Strategy>
struct distance
<
    Point, Linestring, Strategy,
    point_tag, linestring_tag, strategy_tag_distance_point_point,
    false
>
{

    static inline typename return_type<Strategy, Point, typename point_type<Linestring>::type>::type
    apply(Point const& point,
          Linestring const& linestring,
          Strategy const& strategy)
    {
        typedef typename detail::distance::default_ps_strategy
                    <
                        Point,
                        typename point_type<Linestring>::type,
                        Strategy
                    >::type ps_strategy_type;

        return detail::distance::point_to_range
            <
                Point, Linestring, closed, Strategy, ps_strategy_type
            >::apply(point, linestring, strategy, ps_strategy_type());
    }
};


// Point-line version 2, where point-segment strategy is specified
template <typename Point, typename Linestring, typename Strategy>
struct distance
<
    Point, Linestring, Strategy,
    point_tag, linestring_tag, strategy_tag_distance_point_segment,
    false
>
{
    static inline typename return_type<Strategy, Point, typename point_type<Linestring>::type>::type
    apply(Point const& point,
          Linestring const& linestring,
          Strategy const& strategy)
    {
        typedef typename strategy::distance::services::strategy_point_point<Strategy>::type pp_strategy_type;
        return detail::distance::point_to_range
            <
                Point, Linestring, closed, pp_strategy_type, Strategy
            >::apply(point, linestring, pp_strategy_type(), strategy);
    }
};

// Point-ring , where point-segment strategy is specified
template <typename Point, typename Ring, typename Strategy>
struct distance
<
    Point, Ring, Strategy,
    point_tag, ring_tag, strategy_tag_distance_point_point,
    false
>
{
    typedef typename return_type<Strategy, Point, typename point_type<Ring>::type>::type return_type;

    static inline return_type apply(Point const& point,
            Ring const& ring,
            Strategy const& strategy)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Point,
                typename point_type<Ring>::type,
                Strategy
            >::type ps_strategy_type;

        std::pair<return_type, bool>
            dc = detail::distance::point_to_ring
            <
                Point, Ring,
                geometry::closure<Ring>::value,
                Strategy, ps_strategy_type
            >::apply(point, ring, strategy, ps_strategy_type());

        return dc.second ? return_type(0) : dc.first;
    }
};


// Point-polygon , where point-point strategy is specified
template <typename Point, typename Polygon, typename Strategy>
struct distance
<
    Point, Polygon, Strategy,
    point_tag, polygon_tag, strategy_tag_distance_point_point,
    false
>
{
    typedef typename return_type<Strategy, Point, typename point_type<Polygon>::type>::type return_type;

    static inline return_type apply(Point const& point,
            Polygon const& polygon,
            Strategy const& strategy)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Point,
                typename point_type<Polygon>::type,
                Strategy
            >::type ps_strategy_type;

        std::pair<return_type, bool>
            dc = detail::distance::point_to_polygon
            <
                Point, Polygon,
                geometry::closure<Polygon>::value,
                Strategy, ps_strategy_type
            >::apply(point, polygon, strategy, ps_strategy_type());

        return dc.second ? return_type(0) : dc.first;
    }
};


// Point-polygon , where point-segment strategy is specified
template <typename Point, typename Polygon, typename Strategy>
struct distance
<
    Point, Polygon, Strategy,
    point_tag, polygon_tag, strategy_tag_distance_point_segment,
    false
>
{
    typedef typename return_type<Strategy, Point, typename point_type<Polygon>::type>::type return_type;

    static inline return_type apply(Point const& point,
            Polygon const& polygon,
            Strategy const& strategy)
    {
        typedef typename strategy::distance::services::strategy_point_point
            <
                Strategy
            >::type pp_strategy_type;

        std::pair<return_type, bool>
            dc = detail::distance::point_to_polygon
            <
                Point, Polygon,
                geometry::closure<Polygon>::value,
                pp_strategy_type, Strategy
            >::apply(point, polygon, pp_strategy_type(), strategy);

        return dc.second ? return_type(0) : dc.first;
    }
};


// Point-segment version 1, with point-point strategy
template <typename Point, typename Segment, typename Strategy>
struct distance
<
    Point, Segment, Strategy,
    point_tag, segment_tag, strategy_tag_distance_point_point,
    false
> : detail::distance::point_to_segment<Point, Segment, Strategy>
{};

// Point-segment version 2, with point-segment strategy
template <typename Point, typename Segment, typename Strategy>
struct distance
<
    Point, Segment, Strategy,
    point_tag, segment_tag, strategy_tag_distance_point_segment,
    false
>
{
    static inline typename return_type<Strategy, Point, typename point_type<Segment>::type>::type
    apply(Point const& point,
          Segment const& segment,
          Strategy const& strategy)
    {

        typename point_type<Segment>::type p[2];
        geometry::detail::assign_point_from_index<0>(segment, p[0]);
        geometry::detail::assign_point_from_index<1>(segment, p[1]);
        return strategy.apply(point, p[0], p[1]);
    }
};



// dispatch for segment-segment distances
template <typename Segment1, typename Segment2, typename Strategy>
struct distance
    <
         Segment1, Segment2, Strategy, segment_tag, segment_tag,
         strategy_tag_distance_point_segment, false
    >
    : detail::distance::segment_to_segment<Segment1, Segment2, Strategy>
{};


template <typename Segment1, typename Segment2, typename Strategy>
struct distance
    <
         Segment1, Segment2, Strategy, segment_tag, segment_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Segment1>::type,
            typename point_type<Segment2>::type
        >::type
    apply(Segment1 const& segment1, Segment2 const& segment2, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Segment1, Segment2, Strategy
            >::type strategy_type;
        return detail::distance::segment_to_segment
            <
                Segment1, Segment2, strategy_type
            >::apply(segment1, segment2, strategy_type());
    }
};


// dispatch for linestring-segment distances
template <typename Linestring, typename Segment, typename Strategy>
struct distance
    <
        Linestring, Segment, Strategy, linestring_tag, segment_tag,
        strategy_tag_distance_point_segment, false
    >
        : detail::distance::range_to_segment
            <
                Linestring, Segment, closure<Linestring>::value, Strategy
            >
{};

template <typename Linestring, typename Segment, typename Strategy>
struct distance
    <
         Linestring, Segment, Strategy, linestring_tag, segment_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring>::type,
            typename point_type<Segment>::type
        >::type
    apply(Linestring const& linestring, Segment const& segment, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring, Segment, Strategy
            >::type strategy_type;

        return detail::distance::range_to_segment
            <
                Linestring, Segment, closure<Linestring>::value, strategy_type
            >::apply(linestring, segment, strategy_type());
    }
};


// dispatch for ring-segment distances
template <typename Ring, typename Segment, typename Strategy>
struct distance
    <
        Ring, Segment, Strategy, ring_tag, segment_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_segment
        <
            Ring, Segment, closure<Ring>::value, Strategy
        >
{};

template <typename Ring, typename Segment, typename Strategy>
struct distance
    <
         Ring, Segment, Strategy, ring_tag, segment_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Ring>::type,
            typename point_type<Segment>::type
        >::type
    apply(Ring const& ring, Segment const& segment, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Ring, Segment, Strategy
            >::type strategy_type;
        return detail::distance::range_to_segment
            <
                Ring, Segment, closure<Ring>::value, strategy_type
            >::apply(ring, segment, strategy_type());
    }
};




// dispatch for polygon-segment distances
template <typename Polygon, typename Segment, typename Strategy>
struct distance
    <
        Polygon, Segment, Strategy, polygon_tag, segment_tag,
        strategy_tag_distance_point_segment, false
    >    
    : detail::distance::polygon_to_segment<Polygon, Segment, Strategy>
{};

template<typename Polygon, typename Segment, typename Strategy>
struct distance
    <
         Polygon, Segment, Strategy, polygon_tag, segment_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Polygon>::type,
            typename point_type<Segment>::type
        >::type
    apply(Polygon const& polygon, Segment const& segment, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Polygon, Segment, Strategy
            >::type strategy_type;
        return detail::distance::polygon_to_segment
            <
                Polygon, Segment, strategy_type
            >::apply(polygon, segment, strategy_type());
    }
};


// dispatch for point-box distance
template <typename Point, typename Box, typename Strategy>
struct distance
    <
         Point, Box, Strategy, point_tag, box_tag,
         strategy_tag_distance_point_point, false
    >
    : detail::distance::point_to_box<Point, Box, Strategy>
{};


template <typename Point, typename Box, typename Strategy>
struct distance
    <
         Point, Box, Strategy, point_tag, box_tag,
         strategy_tag_distance_point_segment, false
    >
{
    typedef typename strategy::distance::services::strategy_point_point
        <
            Strategy
        >::type pp_strategy_type;

    static inline typename return_type
        <
            pp_strategy_type,
            Point,
            typename point_type<Box>::type
        >::type
    apply(Point const& point, Box const& box, Strategy const&)
    {
        return detail::distance::point_to_box
            <
                Point, Box, pp_strategy_type
            >::apply(point, box, pp_strategy_type());
    }
};


// dispatch for segment-box distance
template <typename Segment, typename Box, typename Strategy>
struct distance
    <
        Segment, Box, Strategy, segment_tag, box_tag,
        strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Segment>::type,
            typename point_type<Box>::type
        >::type
    apply(Segment const& segment, Box const& box, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Segment, Box, Strategy
            >::type strategy_type;

        return detail::distance::segment_to_box
            <
                Segment, Box, strategy_type
            >::apply(segment, box, strategy_type());
    }
};


template <typename Segment, typename Box, typename Strategy>
struct distance
    <
        Segment, Box, Strategy, segment_tag, box_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::segment_to_box<Segment, Box, Strategy>
{};




} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH

/*!
\brief \brief_calc2{distance} \brief_strategy
\ingroup distance
\details
\details \details_calc{area}. \brief_strategy. \details_strategy_reasons

\tparam Geometry1 \tparam_geometry
\tparam Geometry2 \tparam_geometry
\tparam Strategy \tparam_strategy{Distance}
\param geometry1 \param_geometry
\param geometry2 \param_geometry
\param strategy \param_strategy{distance}
\return \return_calc{distance}
\note The strategy can be a point-point strategy. In case of distance point-line/point-polygon
    it may also be a point-segment strategy.

\qbk{distinguish,with strategy}

\qbk{
[heading Available Strategies]
\* [link geometry.reference.strategies.strategy_distance_pythagoras Pythagoras (cartesian)]
\* [link geometry.reference.strategies.strategy_distance_haversine Haversine (spherical)]
\* [link geometry.reference.strategies.strategy_distance_cross_track Cross track (spherical\, point-to-segment)]
\* [link geometry.reference.strategies.strategy_distance_projected_point Projected point (cartesian\, point-to-segment)]
\* more (currently extensions): Vincenty\, Andoyer (geographic)
}
 */

/*
Note, in case of a Compilation Error:
if you get:
 - "Failed to specialize function template ..."
 - "error: no matching function for call to ..."
for distance, it is probably so that there is no specialization
for return_type<...> for your strategy.
*/
template <typename Geometry1, typename Geometry2, typename Strategy>
inline typename strategy::distance::services::return_type
                <
                    Strategy,
                    typename point_type<Geometry1>::type,
                    typename point_type<Geometry2>::type
                >::type
distance(Geometry1 const& geometry1,
         Geometry2 const& geometry2,
         Strategy const& strategy)
{
    concept::check<Geometry1 const>();
    concept::check<Geometry2 const>();

    detail::throw_on_empty_input(geometry1);
    detail::throw_on_empty_input(geometry2);

    return dispatch::distance
               <
                   Geometry1,
                   Geometry2,
                   Strategy
               >::apply(geometry1, geometry2, strategy);
}


/*!
\brief \brief_calc2{distance}
\ingroup distance
\details The default strategy is used, corresponding to the coordinate system of the geometries
\tparam Geometry1 \tparam_geometry
\tparam Geometry2 \tparam_geometry
\param geometry1 \param_geometry
\param geometry2 \param_geometry
\return \return_calc{distance}

\qbk{[include reference/algorithms/distance.qbk]}
 */
template <typename Geometry1, typename Geometry2>
inline typename default_distance_result<Geometry1, Geometry2>::type distance(
                Geometry1 const& geometry1, Geometry2 const& geometry2)
{
    concept::check<Geometry1 const>();
    concept::check<Geometry2 const>();

    return distance(geometry1, geometry2,
                    typename detail::distance::default_strategy<Geometry1, Geometry2>::type());
}

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALTERNATE_DISTANCE

#endif // BOOST_GEOMETRY_ALGORITHMS_DISTANCE_BASIC_HPP
