// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP

#include <boost/range.hpp>

#include <boost/geometry/core/tags.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/core/point_type.hpp>

#include <boost/geometry/geometries/segment.hpp>

#include <boost/geometry/algorithms/not_implemented.hpp>

#include <boost/geometry/views/closeable_view.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{

// forward definitions
template <typename Geometry, typename GeometryTag>
struct split_to_segments_dispatch
    : public not_implemented<Geometry>
{};


template <typename Geometry>
struct split_to_segments
    : split_to_segments_dispatch<Geometry, typename tag<Geometry>::type>
{};



// implementation
template
<
    typename Range,
    closure_selector Closure
>
struct range_to_segments
{
    typedef typename closeable_view<Range const, Closure>::type View;

    template <typename OutputIterator>
    static inline
    OutputIterator apply(Range const& range, OutputIterator oit)
    {
        BOOST_ASSERT( boost::size(range) > 1 );

        View view(range);

        typedef geometry::model::segment
            <
                typename point_type<Range>::type
            > Segment;

        BOOST_AUTO_TPL(it1, boost::begin(view));
        BOOST_AUTO_TPL(it2, boost::begin(view));

        for (++it2; it2 != boost::end(view); ++it1, ++it2)
        {
            *oit++ = Segment(*it1, *it2);
        }

        return oit;
    }
};



template <typename Polygon, closure_selector Closure>
struct polygon_to_segments
{
    typedef typename ring_type<Polygon>::type Ring;

    template <typename OutputIterator>
    static inline
    OutputIterator apply(Polygon const& polygon, OutputIterator oit)
    {
        oit = range_to_segments
            <
                Ring, Closure
            >::apply(geometry::exterior_ring(polygon), oit);

        BOOST_AUTO_TPL(it, boost::begin(geometry::interior_rings(polygon)));
        for (; it != boost::end(geometry::interior_rings(polygon)); ++it)
        {
            oit = range_to_segments
                <
                    Ring, Closure
                >::apply(*it, oit);            
        }

        return oit;
    }
};



template <typename MultiRange, closure_selector Closure>
struct multi_range_to_segments
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(MultiRange const& multirange, OutputIterator oit)
    {
        BOOST_AUTO_TPL(it, boost::begin(multirange));
        for (; it != boost::end(multirange); ++it)
        {
            oit = split_to_segments
                <
                    typename boost::range_value<MultiRange>::type
                >::apply(*it, oit);
        }

        return oit;
    }
};






// dispatch specializations
template <typename LineString>
struct split_to_segments_dispatch<LineString, linestring_tag>
    : range_to_segments<LineString, closed>
{};

template <typename Ring>
struct split_to_segments_dispatch<Ring, ring_tag>
    : range_to_segments<Ring, geometry::closure<Ring>::value>
{};

template <typename Polygon>
struct split_to_segments_dispatch<Polygon, polygon_tag>
    : polygon_to_segments<Polygon, geometry::closure<Polygon>::value>
{};

template <typename MultiLineString>
struct split_to_segments_dispatch<MultiLineString, multi_linestring_tag>
    : multi_range_to_segments<MultiLineString, closed>
{};

template <typename MultiPolygon>
struct split_to_segments_dispatch<MultiPolygon, multi_polygon_tag>
    : multi_range_to_segments
        <
           MultiPolygon,
           geometry::closure
               <
                   typename boost::range_value<MultiPolygon>::type
               >::value
        >
{};



}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP
