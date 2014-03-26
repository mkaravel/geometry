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

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_GEOMETRY_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_GEOMETRY_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <boost/range.hpp>
#include <boost/typeof/typeof.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/reverse_dispatch.hpp>
#include <boost/geometry/core/tag_cast.hpp>

#include <boost/geometry/geometries/concepts/check.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/default_distance_result.hpp>
#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <boost/geometry/views/closeable_view.hpp>

#include <boost/geometry/algorithms/detail/distance/default_strategies.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// To avoid spurious namespaces here:
using strategy::distance::services::return_type;


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
        >::type comparable_pp_strategy;

    typedef typename strategy::distance::services::comparable_type
        <
            PSStrategy
        >::type comparable_ps_strategy;

    typedef typename strategy::distance::services::get_comparable
        <
            PPStrategy
        > get_comparable_pp;

    typedef typename strategy::distance::services::get_comparable
        <
            PSStrategy
        > get_comparable_ps;
 
    static inline return_type apply(Point const& point, Range const& range,
            PPStrategy const& pp_strategy, PSStrategy const& ps_strategy)
    {
        comparable_pp_strategy epp_strategy = get_comparable_pp::apply(pp_strategy);
        comparable_ps_strategy eps_strategy = get_comparable_ps::apply(ps_strategy);

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
                comparable_pp_strategy,
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
                comparable_ps_strategy,
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
        >::type comparable_pp_strategy;

    typedef typename strategy::distance::services::comparable_type
        <
            PSStrategy
        >::type comparable_ps_strategy;

    typedef typename strategy::distance::services::get_comparable
        <
            PPStrategy
        > get_comparable_pp;

    typedef typename strategy::distance::services::get_comparable
        <
            PSStrategy
        > get_comparable_ps;

    static inline distance_containment apply(Point const& point,
                Polygon const& polygon,
                PPStrategy const& pp_strategy, PSStrategy const& ps_strategy)
    {
        comparable_pp_strategy cpp_strategy = get_comparable_pp::apply(pp_strategy);
        comparable_ps_strategy cps_strategy = get_comparable_ps::apply(ps_strategy);

        // Check distance to all rings
        typedef point_to_ring
            <
                Point,
                typename ring_type<Polygon>::type,
                Closure,
                comparable_pp_strategy,
                comparable_ps_strategy
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
                comparable_ps_strategy,
                PSStrategy,
                Point,
                typename point_type<Polygon>::type
            >::apply(dc.first);

        return std::make_pair(rd, dc.second);
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL




#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

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
        P1, P2, Strategy, point_tag, point_tag, 
        strategy_tag_distance_point_segment, false
    >
{
    typedef typename return_type<Strategy, P1, P2>::type return_type;

    static inline return_type apply(P1 const& p1, P2 const& p2,
                                    Strategy const&)
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

// Point-ring , where point-point strategy is specified
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

// Point-ring , where point-segment strategy is specified
template <typename Point, typename Ring, typename Strategy>
struct distance
<
    Point, Ring, Strategy,
    point_tag, ring_tag, strategy_tag_distance_point_segment,
    false
>
{
    typedef typename return_type<Strategy, Point, typename point_type<Ring>::type>::type return_type;

    static inline return_type apply(Point const& point,
            Ring const& ring,
            Strategy const& strategy)
    {
        typedef typename strategy::distance::services::strategy_point_point
            <
                Strategy
            >::type pp_strategy_type;

        std::pair<return_type, bool>
            dc = detail::distance::point_to_ring
            <
                Point, Ring,
                geometry::closure<Ring>::value,
                pp_strategy_type, Strategy
            >::apply(point, ring, pp_strategy_type(), strategy);

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
    Point, Polygon, Strategy, point_tag, polygon_tag, 
    strategy_tag_distance_point_segment, false
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



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_GEOMETRY_HPP
