// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP

#include <boost/geometry/algorithms/detail/distance/get_points.hpp>


namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{



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

#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP
