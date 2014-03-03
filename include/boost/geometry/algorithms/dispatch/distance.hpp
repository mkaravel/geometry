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



//
// dispatch for point-to-geometry combinations
//


// Point-point
template <typename P1, typename P2, typename Strategy>
struct distance
    <
        P1, P2, Strategy,
        point_tag, point_tag, strategy_tag_distance_point_point,
        pointlike_tag, pointlike_tag, single_tag, single_tag,
        false, false, false
    >
    : detail::distance::point_to_point<P1, P2, Strategy>
{};


// Point-point with the point-segment strategy passed
template <typename P1, typename P2, typename Strategy>
struct distance
    <
        P1, P2, Strategy,
        point_tag, point_tag, strategy_tag_distance_point_segment,
        pointlike_tag, pointlike_tag, single_tag, single_tag,
        false, false, false
    >
{
    typedef typename return_type<Strategy, P1, P2>::type return_type;

    static inline return_type apply(P1 const& p1, P2 const& p2,
                                    Strategy const&)
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
    pointlike_tag, linear_tag, single_tag, single_tag,
    false, false, false
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
    pointlike_tag, linear_tag, single_tag, single_tag,
    false, false, false
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
    pointlike_tag, areal_tag, single_tag, single_tag,
    false, false, false
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
    pointlike_tag, areal_tag, single_tag, single_tag,
    false, false, false
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


// point-multigeometry
template
<
    typename Point, typename MultiGeometry, typename Strategy,
    typename MultiGeometryTag, /*typename StrategyTag,*/
    typename MultiTypeTag
>
struct distance
    <
        Point, MultiGeometry, Strategy,
        point_tag, MultiGeometryTag, strategy_tag_distance_point_segment,
        pointlike_tag, MultiTypeTag, single_tag, multi_tag,
        false, false, false
    > : detail::distance::distance_single_to_multi
        <
            Point, MultiGeometry, Strategy
        >
{};



// Point-segment version 1, with point-point strategy
template <typename Point, typename Segment, typename Strategy>
struct distance
<
    Point, Segment, Strategy,
    point_tag, segment_tag, strategy_tag_distance_point_point,
    pointlike_tag, linear_tag, single_tag, single_tag,
    false, true, false
> : detail::distance::point_to_segment<Point, Segment, Strategy>
{};


// Point-segment version 2, with point-segment strategy
template <typename Point, typename Segment, typename Strategy>
struct distance
<
    Point, Segment, Strategy,
    point_tag, segment_tag, strategy_tag_distance_point_segment,
    pointlike_tag, linear_tag, single_tag, single_tag,
    false, true, false
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
    pointlike_tag, areal_tag, single_tag, single_tag,
    false, false, false
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
         strategy_tag_distance_point_point,
         pointlike_tag, areal_tag, single_tag, single_tag,
         false, true, false
    >
    : detail::distance::point_to_box<Point, Box, Strategy>
{};


template <typename Point, typename Box, typename Strategy>
struct distance
    <
         Point, Box, Strategy, point_tag, box_tag,
         strategy_tag_distance_point_segment,
         pointlike_tag, areal_tag, single_tag, single_tag,
         false, true, false
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
// dispatch for linear-to-linear distances
//


// linear-to-linear but not for segments
template
<
    typename Linear1,
    typename Linear2,
    typename Strategy,
    typename Tag1,
    typename Tag2,
    typename SingleMultiTag1,
    typename SingleMultiTag2
>
struct distance
    <
         Linear1, Linear2, Strategy, Tag1, Tag2,
         strategy_tag_distance_point_segment,
         linear_tag, linear_tag, SingleMultiTag1, SingleMultiTag2,
         false, false, false
    >
    : detail::distance::range_to_range
        <
            Linear1, Linear2, Strategy,
            boost::is_same<SingleMultiTag1, multi_tag>::value,
            boost::is_same<SingleMultiTag2, multi_tag>::value
        >
{};


// linestring-segment
template <typename Linestring, typename Segment, typename Strategy>
struct distance
    <
        Linestring, Segment, Strategy, linestring_tag, segment_tag,
        strategy_tag_distance_point_segment,
        linear_tag, linear_tag, single_tag, single_tag,
        false, true, false
    >
        : detail::distance::range_to_segment
            <
                Linestring, Segment, closed, Strategy
            >
{};


// segment-segment
template <typename Segment1, typename Segment2, typename Strategy>
struct distance
    <
        Segment1, Segment2, Strategy, segment_tag, segment_tag,
        strategy_tag_distance_point_segment,
        linear_tag, linear_tag, single_tag, single_tag,
        true, true, false
    >
    : detail::distance::segment_to_segment<Segment1, Segment2, Strategy>
{};


// the multilinestring-segment combination has been implemented under:
// multigeometry-segment



//
// dispatch for linear-to-areal distances
//




// linear-areal
template
<
    typename Linear,
    typename Areal,
    typename Strategy,
    typename Tag1,
    typename Tag2,
    typename SingleMultiTag1,
    typename SingleMultiTag2
>
struct distance
    <
        Linear, Areal, Strategy, Tag1, Tag2,
        strategy_tag_distance_point_segment,
        linear_tag, areal_tag, SingleMultiTag1, SingleMultiTag2,
        false, false, false
    >
    : detail::distance::range_to_range
        <
            Linear, Areal, Strategy,
            boost::is_same<SingleMultiTag1, multi_tag>::value,
            !boost::is_same<Tag2, ring_tag>::value
        >
{};


// areal-linear
template
<
    typename Linear,
    typename Areal,
    typename Strategy,
    typename Tag1,
    typename Tag2,
    typename SingleMultiTag1,
    typename SingleMultiTag2
>
struct distance
    <
        Areal, Linear, Strategy, Tag1, Tag2,
        strategy_tag_distance_point_segment,
        areal_tag, linear_tag, SingleMultiTag1, SingleMultiTag2,
        false, false, false
    >
    : detail::distance::range_to_range
        <
            Areal, Linear, Strategy,
            !boost::is_same<Tag1, ring_tag>::value,
            boost::is_same<SingleMultiTag2, multi_tag>::value
        >
{};


// polygon-segment
template <typename Polygon, typename Segment, typename Strategy>
struct distance
    <
        Polygon, Segment, Strategy, polygon_tag, segment_tag,
        strategy_tag_distance_point_segment,
        areal_tag, linear_tag, single_tag, single_tag,
        false, true, false
    >    
    : detail::distance::polygon_to_segment<Polygon, Segment, Strategy>
{};


// the segment-box combination is implemented below
// the multipolygon-segment combination is implemented as part of the
// multigeometry-segment combination

// MK::missing ring-segment, linestring-box




//
// dispatch for areal-to-areal distances
//




// areal-areal
template
<
    typename Areal1,
    typename Areal2,
    typename Strategy,
    typename Tag1,
    typename Tag2,
    typename SingleMultiTag1,
    typename SingleMultiTag2
>
struct distance
    <
        Areal1, Areal2, Strategy, Tag1, Tag2,
        strategy_tag_distance_point_segment,
        areal_tag, areal_tag, SingleMultiTag1, SingleMultiTag2,
        false, false, false
    >
    : detail::distance::range_to_range
        <
            Areal1, Areal2, Strategy,
            !boost::is_same<Tag1, ring_tag>::value,
            !boost::is_same<Tag2, ring_tag>::value
        >
{};


// MK::missing geometry-box combinations



//
// dispatch for multipoint-to-geometry distances
//



// multipoint-multipoint
template <typename MultiPoint1, typename MultiPoint2, typename Strategy>
struct distance
    <
        MultiPoint1, MultiPoint2, Strategy,
        multi_point_tag, multi_point_tag,
        strategy_tag_distance_point_point,
        pointlike_tag, pointlike_tag, multi_tag, multi_tag,
        false, false, false
    > : detail::distance::multipoint_to_multipoint
        <
            MultiPoint1, MultiPoint2, Strategy
        >
{};


template <typename MultiPoint1, typename MultiPoint2, typename Strategy>
struct distance
    <
        MultiPoint1, MultiPoint2, Strategy,
        multi_point_tag, multi_point_tag,
        strategy_tag_distance_point_segment,
        pointlike_tag, pointlike_tag, multi_tag, multi_tag,
        false, false, false
    > : detail::distance::multipoint_to_multipoint
        <
            MultiPoint1, MultiPoint2, Strategy
        >
{};


// multipoint-linear
template
<
    typename MultiPoint,
    typename Linear,
    typename Strategy,
    typename Tag,
    typename SingleMultiTag
>
struct distance
    <
        MultiPoint, Linear, Strategy, multi_point_tag, Tag,
        strategy_tag_distance_point_segment,
        pointlike_tag, linear_tag, multi_tag, SingleMultiTag,
        false, false, false
    > : detail::distance::multipoint_to_linear
        <
            MultiPoint, Linear, Strategy
        >
{};


// linear-multipoint
template
<
    typename MultiPoint,
    typename Linear,
    typename Strategy,
    typename Tag,
    typename SingleMultiTag
>
struct distance
    <
        Linear, MultiPoint, Strategy, Tag, multi_point_tag,
        strategy_tag_distance_point_segment,
        linear_tag, pointlike_tag, SingleMultiTag, multi_tag,
        false, false, false
    > : detail::distance::multipoint_to_linear
        <
            MultiPoint, Linear, Strategy
        >
{};




//
// dispatch for multigeometry-segment
//




// multigeometry-segment
template
<
    typename MultiGeometry,
    typename Segment,
    typename Strategy,
    typename Tag,
    typename TypeTag
>
struct distance
    <
        MultiGeometry, Segment, Strategy,
        Tag, segment_tag, strategy_tag_distance_point_segment,
        TypeTag, linear_tag, multi_tag, single_tag,
        false, true, false
    > : detail::distance::distance_single_to_multi
        <
            Segment, MultiGeometry, Strategy
        >
{};

// MK::missing multipoint-areal




//
// dispatch for segment-to-geometry distances
//


// segment-ring
template <typename Segment, typename Ring, typename Strategy>
struct distance
    <
        Segment, Ring, Strategy, segment_tag, ring_tag,
        strategy_tag_distance_point_segment,
        linear_tag, areal_tag, single_tag, single_tag,
        true, false, false
    >
    : detail::distance::range_to_segment
        <
            Ring, Segment, closure<Ring>::value, Strategy
        >
{};


// segment-box
template <typename Segment, typename Box, typename Strategy>
struct distance
    <
        Segment, Box, Strategy, segment_tag, box_tag,
        strategy_tag_distance_point_segment,
        linear_tag, areal_tag, single_tag, single_tag,
        true, true, false
    >
    : detail::distance::segment_to_box<Segment, Box, Strategy>
{};


// MK::missing: ring-box, box-box




//
// dispatch for point-point strategy for linear and/or areal geometries
//




// geometry-geometry with PP strategy
// in cases where the natural strategy is the PP strategy, there exist
// specific specializations
template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy,
    typename Tag1,
    typename Tag2,
    typename TypeTag1,
    typename TypeTag2,
    typename SingleMultiTag1,
    typename SingleMultiTag2,
    bool SegmentOrBox1,
    bool SegmentOrBox2
>
struct distance
    <
        Geometry1, Geometry2, Strategy, Tag1, Tag2,
        strategy_tag_distance_point_point,
        TypeTag1, TypeTag2, SingleMultiTag1, SingleMultiTag2,
        SegmentOrBox1, SegmentOrBox2, false
    >
{
    static inline typename return_type
        <
            Strategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type apply(Geometry1 const& geometry1,
                      Geometry2 const& geometry2,
                      Strategy const&)
    {
        typedef typename detail::distance::default_ps_strategy
            <
                Geometry1, Geometry2, Strategy
            >::type PS_Strategy;

        return distance
            <
                Geometry1, Geometry2, PS_Strategy, Tag1, Tag2,
                strategy_tag_distance_point_segment,
                TypeTag1, TypeTag2, SingleMultiTag1, SingleMultiTag2,
                SegmentOrBox1, SegmentOrBox2, false
            >::apply(geometry1, geometry2, PS_Strategy());
    }
};



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DISPATCH_DISTANCE_HPP
