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

#include <boost/geometry/algorithms/detail/distance/default_strategies.hpp>
#include <boost/geometry/algorithms/detail/distance/point_to_box.hpp>

#include <boost/geometry/strategies/cartesian/distance_comparable_to_regular.hpp>


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


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POINT_TO_GEOMETRY_HPP
