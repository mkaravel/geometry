// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP

#include <boost/geometry/algorithms/detail/distance/split_to_segments.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{



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

#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP
