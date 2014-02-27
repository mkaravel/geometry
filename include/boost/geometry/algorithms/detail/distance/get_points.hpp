// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP

#include <algorithm>

#include <boost/geometry/core/tags.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/core/point_type.hpp>

#include <boost/geometry/algorithms/not_implemented.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// generic definitions

template <typename Geometry, typename GeometryTag>
struct get_points_dispatch
    : public not_implemented<Geometry>
{};


template <typename Geometry>
struct get_points
    : get_points_dispatch<Geometry, typename tag<Geometry>::type>
{};





// implementation and specializations

template <typename Point>
struct get_points_point
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(Point const& point, OutputIterator oit)
    {
        *oit++ = point;
        return oit;
    }
};


template <typename Segment>
struct get_points_segment
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(Segment const& segment, OutputIterator oit)
    {
        typename point_type<Segment>::type p;
        detail::assign_point_from_index<0>(segment, p);
        *oit++ = p;
        detail::assign_point_from_index<1>(segment, p);
        *oit++ = p;
        return oit;
    }
};


template <typename Range>
struct get_points_range
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(Range const& range, OutputIterator oit)
    {
        return std::copy(boost::begin(range), boost::end(range), oit);
    }
};


template <typename Polygon>
struct get_points_polygon
{
    typedef typename ring_type<Polygon>::type Ring;

    template <typename OutputIterator>
    static inline
    OutputIterator apply(Polygon const& polygon, OutputIterator oit)
    {
        oit = get_points_range<Ring>::apply(geometry::exterior_ring(polygon),
                                            oit);

        BOOST_AUTO_TPL(it, boost::begin(geometry::interior_rings(polygon)));
        for (; it != boost::end(geometry::interior_rings(polygon)); ++it)
        {
            oit = get_points_range<Ring>::apply(*it, oit);            
        }

        return oit;
    }
};


template <typename MultiRange>
struct get_points_multi_range
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(MultiRange const& multirange, OutputIterator oit)
    {
        BOOST_AUTO_TPL(it, boost::begin(multirange));
        for (; it != boost::end(multirange); ++it)
        {
            oit = get_points
                <
                    typename boost::range_value<MultiRange>::type
                >::apply(*it, oit);
        }

        return oit;
    }
};




// dispatch specializations
template <typename Point>
struct get_points_dispatch<Point, point_tag>
    : get_points_point<Point>
{};

template <typename Segment>
struct get_points_dispatch<Segment, segment_tag>
    : get_points_segment<Segment>
{};

template <typename LineString>
struct get_points_dispatch<LineString, linestring_tag>
    : get_points_range<LineString>
{};

template <typename Ring>
struct get_points_dispatch<Ring, ring_tag>
    : get_points_range<Ring>
{};

template <typename Polygon>
struct get_points_dispatch<Polygon, polygon_tag>
    : get_points_polygon<Polygon>
{};

template <typename MultiPoint>
struct get_points_dispatch<MultiPoint, multi_point_tag>
    : get_points_range<MultiPoint>
{};

template <typename MultiLineString>
struct get_points_dispatch<MultiLineString, multi_linestring_tag>
    : get_points_multi_range<MultiLineString>
{};

template <typename MultiPolygon>
struct get_points_dispatch<MultiPolygon, multi_polygon_tag>
    : get_points_multi_range<MultiPolygon>
{};




}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP
