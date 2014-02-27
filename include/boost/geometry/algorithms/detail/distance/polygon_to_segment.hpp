// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POLYGON_TO_SEGMENT_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POLYGON_TO_SEGMENT_HPP

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

#include <boost/geometry/algorithms/detail/distance/point_to_geometry.hpp>
#include <boost/geometry/algorithms/detail/distance/range_to_segment.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


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


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POLYGON_TO_SEGMENT_HPP
