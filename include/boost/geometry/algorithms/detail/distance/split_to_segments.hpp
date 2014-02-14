// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP

#include <boost/geometry/geometries/segment.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


template <typename LineString>
struct linestring_to_segments
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(LineString const& ls, OutputIterator oit)
    {
        typedef geometry::model::segment
            <
                typename point_type<LineString>::type
            > Segment;

        BOOST_AUTO_TPL(it1, boost::begin(ls));
        BOOST_AUTO_TPL(it2, boost::begin(ls));

        for (++it2; it2 != boost::end(ls); ++it1, ++it2)
        {
            *oit++ = Segment(*it1, *it2);
        }

        return oit;
    }
};


template <typename MultiLineString>
struct multilinestring_to_segments
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(MultiLineString const& mls, OutputIterator oit)
    {
        BOOST_AUTO_TPL(it, boost::begin(mls));
        for (; it != boost::end(mls); ++it)
        {
            oit = linestring_to_segments
                <
                    typename boost::range_value<MultiLineString>::type
                >::apply(*it, oit);
        }

        return oit;
    }
};






template <typename Geometry, typename GeometryTag>
struct split_to_segments_dispatch
    : public not_implemented<Geometry>
{};


template <typename LineString>
struct split_to_segments_dispatch<LineString, linestring_tag>
    : linestring_to_segments<LineString>
{};


template <typename MultiLineString>
struct split_to_segments_dispatch<MultiLineString, multi_linestring_tag>
    : multilinestring_to_segments<MultiLineString>
{};








template <typename Geometry>
struct split_to_segments
    : split_to_segments_dispatch<Geometry, typename tag<Geometry>::type>
{};



}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP
