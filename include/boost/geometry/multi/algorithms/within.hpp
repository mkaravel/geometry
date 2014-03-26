// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.
// Copyright (c) 2013 Adam Wulkiewicz, Lodz, Poland.

// This file was modified by Oracle on 2013.
// Modifications copyright (c) 2013, Oracle and/or its affiliates.

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_WITHIN_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_WITHIN_HPP


#include <boost/range.hpp>

#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/multi/core/closure.hpp>
#include <boost/geometry/multi/core/point_order.hpp>
#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/geometries/concepts/check.hpp>

#include <boost/geometry/multi/algorithms/detail/within/point_in_geometry.hpp>

namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace within
{


template
<
    typename Geometry,
    typename MultiGeometry,
    typename Strategy,
    typename Policy
>
struct geometry_multi_within_code
{
    static inline int apply(Geometry const& geometry,
            MultiGeometry const& multi,
            Strategy const& strategy)
    {
        for (typename boost::range_iterator<MultiGeometry const>::type it
                    = boost::begin(multi);
            it != boost::end(multi);
            ++it)
        {
            // Geometry coding on multi: 1 (within) if within one of them;
            // 0 (touch) if on border of one of them
            int const code = Policy::apply(geometry, *it, strategy);
            if (code != -1)
            {
                return code;
            }
        }
        return -1;
    }
};


}} // namespace detail::within
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

template <typename Point, typename MultiPolygon>
struct within<Point, MultiPolygon, point_tag, multi_polygon_tag>
{
    template <typename Strategy>
    static inline bool apply(Point const& point,
                             MultiPolygon const& multi_polygon,
                             Strategy const& strategy)
    {
        return detail::within::geometry_multi_within_code
            <
                Point,
                MultiPolygon,
                Strategy,
                detail_dispatch::within::point_in_geometry
                    <
                        typename boost::range_value<MultiPolygon>::type
                    >
            >::apply(point, multi_polygon, strategy) == 1;
    }
};

template <typename Point, typename MultiLinestring>
struct within<Point, MultiLinestring, point_tag, multi_linestring_tag>
{
    template <typename Strategy>
    static inline bool apply(Point const& point,
                             MultiLinestring const& multi_linestring,
                             Strategy const& strategy)
    {
        return detail::within::point_in_geometry(point, multi_linestring, strategy) == 1;
    }
};

template <typename Point, typename MultiPoint>
struct within<Point, MultiPoint, point_tag, multi_point_tag>
{
    template <typename Strategy>
    static inline bool apply(Point const& point,
                             MultiPoint const& multi_point,
                             Strategy const& strategy)
    {
        return detail::within::point_in_geometry(point, multi_point, strategy) == 1;
    }
};

} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_WITHIN_HPP
