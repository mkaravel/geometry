// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_GET_SEGMENTS_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_GET_SEGMENTS_HPP

#include <iterator>

#include <boost/range.hpp>

#include <boost/geometry/core/exterior_ring.hpp>
#include <boost/geometry/core/interior_rings.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/core/ring_type.hpp>
#include <boost/geometry/core/tag.hpp>
#include <boost/geometry/core/tags.hpp>

#include <boost/geometry/geometries/segment.hpp>

#include <boost/geometry/algorithms/not_implemented.hpp>

#include <boost/geometry/views/closeable_view.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{

// forward definitions
template <typename Geometry, typename GeometryTag, typename Segment>
struct get_segments_dispatch
    : public not_implemented<Geometry, Segment>
{};


template
<
    typename Geometry,
    typename Segment = model::segment<typename point_type<Geometry>::type>
>
struct get_segments
    : get_segments_dispatch<Geometry, typename tag<Geometry>::type, Segment>
{};



// implementation
template
<
    typename Range,
    closure_selector Closure,
    typename Segment
>
struct range_to_segments
{
    template <typename RangeIterator, typename OutputIterator>
    static inline
    OutputIterator apply(RangeIterator first, RangeIterator beyond,
                         OutputIterator oit)
    {
        RangeIterator it1 = first;
        RangeIterator it2 = first;

        for (++it2; it2 != beyond; ++it1, ++it2)
        {
            *oit++ = Segment(*it1, *it2);
        }

        return oit;
    }

    template <typename OutputIterator>
    static inline
    OutputIterator apply(Range const& range, OutputIterator oit)
    {
        BOOST_ASSERT( boost::size(range) > 1 );

        typename closeable_view<Range const, Closure>::type view(range);

        return apply(boost::begin(view), boost::end(view), oit);
    }
};



template <typename Polygon, closure_selector Closure, typename Segment>
struct polygon_to_segments
{
    template <typename RingIterator, typename OutputIterator>
    static inline
    OutputIterator apply_to_interior_rings(RingIterator first,
                                           RingIterator beyond,
                                           OutputIterator oit)
    {
        for (RingIterator it = first; it != beyond; ++it)
        {
            oit = range_to_segments
                <
                    typename std::iterator_traits<RingIterator>::value_type,
                    Closure,
                    Segment
                >::apply(*it, oit);            
        }

        return oit;
    }

    template <typename InteriorRings, typename OutputIterator>
    static inline
    OutputIterator apply_to_interior_rings(InteriorRings const& interior_rings,
                                           OutputIterator oit)
    {
        return apply_to_interior_rings(boost::begin(interior_rings),
                                       boost::end(interior_rings),
                                       oit);
    }

    template <typename OutputIterator>
    static inline
    OutputIterator apply(Polygon const& polygon, OutputIterator oit)
    {
        oit = range_to_segments
            <
                typename ring_type<Polygon>::type, Closure, Segment
            >::apply(geometry::exterior_ring(polygon), oit);

        return apply_to_interior_rings(geometry::interior_rings(polygon), oit);
    }
};



template <typename MultiRange, typename Segment>
struct multi_range_to_segments
{
    template <typename MultiIterator, typename OutputIterator>
    static inline
    OutputIterator apply(MultiIterator first,
                         MultiIterator beyond,
                         OutputIterator oit)
    {
        for (MultiIterator it = first; it != beyond; ++it)
        {
            oit = get_segments
                <
                    typename std::iterator_traits<MultiIterator>::value_type,
                    Segment
                >::apply(*it, oit);
        }
        return oit;
    }

    template <typename OutputIterator>
    static inline
    OutputIterator apply(MultiRange const& multirange, OutputIterator oit)
    {
        return apply(boost::begin(multirange), boost::end(multirange), oit);
    }
};






// dispatch specializations
template <typename LineString, typename Segment>
struct get_segments_dispatch<LineString, linestring_tag, Segment>
    : range_to_segments<LineString, closed, Segment>
{};

template <typename Ring, typename Segment>
struct get_segments_dispatch<Ring, ring_tag, Segment>
    : range_to_segments<Ring, geometry::closure<Ring>::value, Segment>
{};

template <typename Polygon, typename Segment>
struct get_segments_dispatch<Polygon, polygon_tag, Segment>
    : polygon_to_segments<Polygon, geometry::closure<Polygon>::value, Segment>
{};

template <typename MultiLineString, typename Segment>
struct get_segments_dispatch<MultiLineString, multi_linestring_tag, Segment>
    : multi_range_to_segments<MultiLineString, Segment>
{};

template <typename MultiPolygon, typename Segment>
struct get_segments_dispatch<MultiPolygon, multi_polygon_tag, Segment>
    : multi_range_to_segments<MultiPolygon, Segment>
{};



}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_GET_SEGMENTS_HPP
