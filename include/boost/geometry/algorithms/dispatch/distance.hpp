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

#ifndef BOOST_GEOMETRY_ALGORITHMS_DISPATCH_DISTANCE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DISPATCH_DISTANCE_HPP

#include <boost/geometry/algorithms/distance.hpp>

#include <boost/range.hpp>
#include <boost/numeric/conversion/bounds.hpp>

#include <boost/geometry/core/closure.hpp>

#include <boost/geometry/geometries/segment.hpp>

#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/views/closeable_view.hpp>
#include <boost/geometry/util/math.hpp>

#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/core/geometry_id.hpp>
#include <boost/geometry/multi/core/point_type.hpp>
#include <boost/geometry/multi/geometries/concepts/check.hpp>

#include <boost/geometry/multi/algorithms/num_points.hpp>
#include <boost/geometry/util/select_coordinate_type.hpp>


// the implementation details
#include <boost/geometry/algorithms/detail/distance/point_to_geometry.hpp>
#include <boost/geometry/algorithms/detail/distance/range_to_range.hpp>
#include <boost/geometry/algorithms/detail/distance/range_to_segment.hpp>
#include <boost/geometry/algorithms/detail/distance/segment_to_box.hpp>
#include <boost/geometry/algorithms/detail/distance/polygon_to_segment.hpp>
#include <boost/geometry/algorithms/detail/distance/single_to_multi.hpp>
#include <boost/geometry/algorithms/detail/distance/multipoint_to_range.hpp>


namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


using strategy::distance::services::return_type;



// If reversal is needed, perform it
template
<
    typename Geometry1, typename Geometry2, typename Strategy,
    typename Tag1, typename Tag2, typename StrategyTag
>
struct distance
<
    Geometry1, Geometry2, Strategy,
    Tag1, Tag2, StrategyTag,
    true
>
    : distance<Geometry2, Geometry1, Strategy, Tag2, Tag1, StrategyTag, false>
{
    typedef typename strategy::distance::services::return_type
                     <
                         Strategy,
                         typename point_type<Geometry2>::type,
                         typename point_type<Geometry1>::type
                     >::type return_type;

    static inline return_type apply(
        Geometry1 const& g1,
        Geometry2 const& g2,
        Strategy const& strategy)
    {
        return distance
            <
                Geometry2, Geometry1, Strategy,
                Tag2, Tag1, StrategyTag,
                false
            >::apply(g2, g1, strategy);
    }
};




//
// dispatch for point-to-geometry combinations
//




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
        P1, P2, Strategy,
        point_tag, point_tag, strategy_tag_distance_point_segment,
        false
    >
{
    typedef typename return_type<Strategy, P1, P2>::type return_type;

    static inline return_type apply(P1 const& p1, P2 const& p2, Strategy&)
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
    Point, Polygon, Strategy,
    point_tag, polygon_tag, strategy_tag_distance_point_segment,
    false
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


// point-multipoint
template
<
    typename Point, typename MultiPoint, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Point, MultiPoint, Strategy,
        point_tag, multi_point_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Point, MultiPoint, Strategy
        >
{};


// point-multilinestring
template
<
    typename Point, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Point, MultiLinestring, Strategy,
        point_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Point, MultiLinestring, Strategy
        >
{};


// point-multipolygon
template
<
    typename Point, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Point, MultiPolygon, Strategy,
        point_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Point, MultiPolygon, Strategy
        >
{};


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


// Point-ring , where point-segment strategy is specified
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


// point-box
template <typename Point, typename Box, typename Strategy>
struct distance
    <
         Point, Box, Strategy, point_tag, box_tag,
         strategy_tag_distance_point_point, false
    >
    : detail::distance::point_to_box<Point, Box, Strategy>
{};


template <typename Point, typename Box, typename Strategy>
struct distance
    <
         Point, Box, Strategy, point_tag, box_tag,
         strategy_tag_distance_point_segment, false
    >
{
    typedef typename strategy::distance::services::strategy_point_point
        <
            Strategy
        >::type pp_strategy_type;

    static inline typename return_type
        <
            pp_strategy_type,
            Point,
            typename point_type<Box>::type
        >::type
    apply(Point const& point, Box const& box, Strategy const&)
    {
        return detail::distance::point_to_box
            <
                Point, Box, pp_strategy_type
            >::apply(point, box, pp_strategy_type());
    }
};




//
// dispatch for linestring-to-geometry distances
//




// linestring-linestring
template <typename Linestring1, typename Linestring2, typename Strategy>
struct distance
    <
        Linestring1, Linestring2, Strategy, linestring_tag, linestring_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Linestring1, Linestring2, Strategy, false, false
        >
{};

template <typename Linestring1, typename Linestring2, typename Strategy>
struct distance
    <
         Linestring1, Linestring2, Strategy, linestring_tag, linestring_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring1>::type,
            typename point_type<Linestring2>::type
        >::type
    apply(Linestring1 const& linestring1, Linestring2 const& linestring2,
          Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring1, Linestring2, Strategy
            >::type strategy_type;
        return detail::distance::range_to_range
            <
                Linestring1, Linestring2, strategy_type, false, false
            >::apply(linestring1, linestring2, strategy_type());
    }
};


// linestring-polygon
template <typename Linestring, typename Polygon, typename Strategy>
struct distance
    <
        Linestring, Polygon, Strategy, linestring_tag, polygon_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
          Linestring, Polygon, Strategy, false, true
        >
{};

template<typename Linestring, typename Polygon, typename Strategy>
struct distance
    <
         Linestring, Polygon, Strategy, linestring_tag, polygon_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring>::type,
            typename point_type<Polygon>::type
        >::type
    apply(Linestring const& linestring, Polygon const& polygon, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring, Polygon, Strategy
            >::type strategy_type;

        return detail::distance::range_to_range
            <
                Linestring, Polygon, strategy_type, false, true
            >::apply(linestring, polygon, strategy_type());
    }
};


// linestring-multipoint
template
<
    typename Linestring, typename MultiPoint, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Linestring, MultiPoint, Strategy,
        linestring_tag, multi_point_tag, StrategyTag, false
    > : detail::distance::multipoint_to_linear
        <
            MultiPoint, Linestring, Strategy
        >
{};


// linestring-multilinestring
template
<
    typename Linestring, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Linestring, MultiLinestring, Strategy,
        linestring_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Linestring, MultiLinestring, Strategy, false, true
        >
{};



// linestring-multipolygon
template
<
    typename Linestring, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Linestring, MultiPolygon, Strategy,
        linestring_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Linestring, MultiPolygon, Strategy, false, true
        >
{};


// linestring-segment
template <typename Linestring, typename Segment, typename Strategy>
struct distance
    <
        Linestring, Segment, Strategy, linestring_tag, segment_tag,
        strategy_tag_distance_point_segment, false
    >
        : detail::distance::range_to_segment
            <
                Linestring, Segment, closure<Linestring>::value, Strategy
            >
{};


template <typename Linestring, typename Segment, typename Strategy>
struct distance
    <
         Linestring, Segment, Strategy, linestring_tag, segment_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring>::type,
            typename point_type<Segment>::type
        >::type
    apply(Linestring const& linestring, Segment const& segment, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring, Segment, Strategy
            >::type strategy_type;

        return detail::distance::range_to_segment
            <
                Linestring, Segment, closure<Linestring>::value, strategy_type
            >::apply(linestring, segment, strategy_type());
    }
};


// linestring-ring
template <typename Linestring, typename Ring, typename Strategy>
struct distance
    <
        Linestring, Ring, Strategy, linestring_tag, ring_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Linestring, Ring, Strategy, false, false
        >
{};

template <typename Linestring, typename Ring, typename Strategy>
struct distance
    <
         Linestring, Ring, Strategy, linestring_tag, ring_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Linestring>::type,
            typename point_type<Ring>::type
        >::type
    apply(Linestring const& linestring, Ring const& ring, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Linestring, Ring, Strategy
            >::type strategy_type;
        return detail::distance::range_to_range
            <
                Linestring, Ring, strategy_type, false, false
            >::apply(linestring, ring, strategy_type());
    }
};


// MK::missing linestring-box




//
// dispatch for polygon-to-geometry distances
//




// polygon-polygon
template <typename Polygon1, typename Polygon2, typename Strategy>
struct distance
    <
        Polygon1, Polygon2, Strategy, polygon_tag, polygon_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_range
        <
            Polygon1, Polygon2, Strategy, true, true
        >
{};

template <typename Polygon1, typename Polygon2, typename Strategy>
struct distance
    <
         Polygon1, Polygon2, Strategy, polygon_tag, polygon_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Polygon1>::type,
            typename point_type<Polygon2>::type
        >::type
    apply(Polygon1 const& polygon1, Polygon2 const& polygon2, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Polygon1, Polygon2, Strategy
            >::type strategy_type;

        return detail::distance::range_to_range
            <
                Polygon1, Polygon2, strategy_type, true, true
            >::apply(polygon1, polygon2, strategy_type());
    }
};


// MK::missing polygon-multipoint


// polygon-multilinestring
template
<
    typename Polygon, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Polygon, MultiLinestring, Strategy,
        polygon_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Polygon, MultiLinestring, Strategy, true, true
        >
{};


// polygon-multipolygon
template
<
    typename Polygon, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        Polygon, MultiPolygon, Strategy,
        polygon_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            Polygon, MultiPolygon, Strategy, true, true
        >
{};


// polygon-segment
template <typename Polygon, typename Segment, typename Strategy>
struct distance
    <
        Polygon, Segment, Strategy, polygon_tag, segment_tag,
        strategy_tag_distance_point_segment, false
    >    
    : detail::distance::polygon_to_segment<Polygon, Segment, Strategy>
{};


template<typename Polygon, typename Segment, typename Strategy>
struct distance
    <
         Polygon, Segment, Strategy, polygon_tag, segment_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Polygon>::type,
            typename point_type<Segment>::type
        >::type
    apply(Polygon const& polygon, Segment const& segment, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Polygon, Segment, Strategy
            >::type strategy_type;
        return detail::distance::polygon_to_segment
            <
                Polygon, Segment, strategy_type
            >::apply(polygon, segment, strategy_type());
    }
};


// MK::missing polygon-ring


// MK::missing polygon-box




//
// dispatch for multipoint-to-geometry distances
//




// multipoint-multipoint
template
<
    typename MultiPoint1, typename MultiPoint2, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPoint1, MultiPoint2, Strategy,
        multi_point_tag, multi_point_tag, StrategyTag, false
    > : detail::distance::multipoint_to_multipoint
        <
            MultiPoint1, MultiPoint2, Strategy
        >
{};


// multipoint-multilinestring
template
<
    typename MultiPoint, typename MultiLinestring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPoint, MultiLinestring, Strategy,
        multi_point_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::multipoint_to_linear
        <
            MultiPoint, MultiLinestring, Strategy
        >
{};


// MK::missing multipoint-multipolygon


// multipoint-segment
template
<
    typename MultiPoint, typename Segment, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPoint, Segment, Strategy,
        multi_point_tag, segment_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi<Segment, MultiPoint, Strategy>
{};


// MK::missing multipoint-ring


// MK::missing multipoint-box




//
// dispatch for multilinestring-to-geometry distances
//




// multilinestring-multilinestring
template
<
    typename MultiLinestring1, typename MultiLinestring2, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiLinestring1, MultiLinestring2, Strategy,
        multi_linestring_tag, multi_linestring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiLinestring1, MultiLinestring2, Strategy, true, true
        >
{};


// multilinestring-multipolygon
template
<
    typename MultiLinestring, typename MultiPolygon, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiLinestring, MultiPolygon, Strategy,
        multi_linestring_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiLinestring, MultiPolygon, Strategy, true, true
        >
{};


// multilinestring-segment
template
<
    typename MultiLinestring, typename Segment, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiLinestring, Segment, Strategy,
        multi_linestring_tag, segment_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Segment, MultiLinestring, Strategy
        >
{};


// multilinestring-ring
template
<
    typename MultiLinestring, typename Ring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiLinestring, Ring, Strategy,
        multi_linestring_tag, ring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiLinestring, Ring, Strategy, true, false
        >
{};


// MK::missing multilinestring-box




//
// dispatch for multipolygon-to-geometry distances
//




// multipolygon-multipolygon
template
<
    typename MultiPolygon1, typename MultiPolygon2, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPolygon1, MultiPolygon2, Strategy,
        multi_polygon_tag, multi_polygon_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiPolygon1, MultiPolygon2, Strategy, true, true
        >
{};


// multipolygon-segment
template
<
    typename MultiPolygon, typename Segment, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPolygon, Segment, Strategy,
        multi_polygon_tag, segment_tag, StrategyTag, false
    > : detail::distance::distance_single_to_multi
        <
            Segment, MultiPolygon, Strategy
        >
{};


// multipolygon-ring
template
<
    typename MultiPolygon, typename Ring, typename Strategy,
    typename StrategyTag
>
struct distance
    <
        MultiPolygon, Ring, Strategy,
        multi_polygon_tag, ring_tag, StrategyTag, false
    > : detail::distance::range_to_range
        <
            MultiPolygon, Ring, Strategy, true, false
        >
{};


// MK::missing multipolygon-box




//
// dispatch for segment-to-geometry distances
//




// segment-segment
template <typename Segment1, typename Segment2, typename Strategy>
struct distance
    <
         Segment1, Segment2, Strategy, segment_tag, segment_tag,
         strategy_tag_distance_point_segment, false
    >
    : detail::distance::segment_to_segment<Segment1, Segment2, Strategy>
{};


template <typename Segment1, typename Segment2, typename Strategy>
struct distance
    <
         Segment1, Segment2, Strategy, segment_tag, segment_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Segment1>::type,
            typename point_type<Segment2>::type
        >::type
    apply(Segment1 const& segment1, Segment2 const& segment2, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Segment1, Segment2, Strategy
            >::type strategy_type;
        return detail::distance::segment_to_segment
            <
                Segment1, Segment2, strategy_type
            >::apply(segment1, segment2, strategy_type());
    }
};


// segment-ring
template <typename Segment, typename Ring, typename Strategy>
struct distance
    <
        Segment, Ring, Strategy, segment_tag, ring_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::range_to_segment
        <
            Ring, Segment, closure<Ring>::value, Strategy
        >
{};


template <typename Segment, typename Ring, typename Strategy>
struct distance
    <
         Segment, Ring, Strategy, segment_tag, ring_tag,
         strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Segment>::type,
            typename point_type<Ring>::type
        >::type
    apply(Segment const& segment, Ring const& ring, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Segment, Ring, Strategy
            >::type strategy_type;
        return detail::distance::range_to_segment
            <
                Ring, Segment, closure<Ring>::value, strategy_type
            >::apply(ring, segment, strategy_type());
    }
};


// segment-box (implemented only for 2D)
template <typename Segment, typename Box, typename Strategy>
struct distance
    <
        Segment, Box, Strategy, segment_tag, box_tag,
        strategy_tag_distance_point_point, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Segment>::type,
            typename point_type<Box>::type
        >::type
    apply(Segment const& segment, Box const& box, Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Segment, Box, Strategy
            >::type strategy_type;

        return detail::distance::segment_to_box
            <
                Segment, Box, strategy_type
            >::apply(segment, box, strategy_type());
    }
};


template <typename Segment, typename Box, typename Strategy>
struct distance
    <
        Segment, Box, Strategy, segment_tag, box_tag,
        strategy_tag_distance_point_segment, false
    >
    : detail::distance::segment_to_box<Segment, Box, Strategy>
{};


// MK::missing: ring-ring, ring-box, box-box



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DISPATCH_DISTANCE_HPP
