// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_HPP

#include <boost/foreach.hpp>
#include <boost/range.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/detail/distance/get_points.hpp>
#include <boost/geometry/algorithms/detail/distance/split_to_segments.hpp>

#include <boost/geometry/index/rtree.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


template <typename TreeRange, typename QueryRange, typename Strategy>
struct range_to_range_rtree
{
    typedef typename boost::range_value<TreeRange>::type RTreeValue;
    typedef typename boost::range_value<QueryRange>::type QueryValue;

    typedef typename point_type<RTreeValue>::type Point1;
    typedef typename point_type<QueryValue>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   

    static inline return_type apply(TreeRange const& t_range,
                                    QueryRange const& q_range,
                                    Strategy const& strategy)
    {
        // create -- packing algorithm
        index::rtree
            <
                RTreeValue, index::linear<8>
            > rt(boost::begin(t_range), boost::end(t_range));


        BOOST_AUTO_TPL(it, boost::begin(q_range));

        QueryValue q_v(*it);

        RTreeValue t_v;
        std::size_t n = rt.query(index::nearest(q_v, 1), &t_v);
        assert( n > 0 );
        return_type min_cd = geometry::distance(t_v, q_v, strategy);

        ++it;
        for (; it != boost::end(q_range); ++it)
        {
            q_v = *it;
            n = rt.query(index::nearest(q_v, 1), &t_v);
            assert( n > 0 );
            return_type cd = geometry::distance(t_v, q_v, strategy);

            if ( cd < min_cd )
            {
                min_cd = cd;
            }
        }

        return min_cd;
    }
};



template <typename PointRange, typename Geometry, typename Strategy>
struct point_range_to_linear_rtree
{
    typedef typename boost::range_value<PointRange>::type Point1;
    typedef typename point_type<Geometry>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   


    static inline return_type apply(PointRange const& point_range,
                                    Geometry const& geometry,
                                    Strategy const& strategy)
    {
        typedef geometry::model::segment<Point2> Segment;

        typedef std::vector<Segment> SegmentRange;
        SegmentRange segments;

        split_to_segments<Geometry>::apply(geometry,
                                           std::back_inserter(segments));

        return range_to_range_rtree
            <
                PointRange, SegmentRange, Strategy
            >::apply(point_range, segments, strategy);
    }
};



template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy,
    bool CollectPoints1,
    bool CollectPoints2
>
struct linear_to_linear_rtree
    : not_implemented<Geometry1, Geometry2>
{};

template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy
>
struct linear_to_linear_rtree<Geometry1, Geometry2, Strategy, true, true>
{
    // the following works with linear geometries seen as ranges of points
    //
    // we compute the r-tree for the points of one range and then,
    // compute nearest points for the segments of the other,
    // ... and ...
    // vice versa.

    typedef typename point_type<Geometry1>::type Point1;
    typedef typename point_type<Geometry2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2,
                                    Strategy const& strategy,
                                    bool check_intersection = true)
    {
        if ( check_intersection && geometry::intersects(geometry1, geometry2) )
        {
            return return_type(0);
        }

        typedef std::vector<Point1> PointRange1;
        typedef std::vector<Point2> PointRange2;

        PointRange1 points1;
        PointRange2 points2;
        get_points<Geometry1>::apply(geometry1, std::back_inserter(points1));
        get_points<Geometry2>::apply(geometry2, std::back_inserter(points2));

        return_type cdist1 = point_range_to_linear_rtree
            <
                PointRange1, Geometry2, Strategy
            >::apply(points1, geometry2, strategy);

        return_type cdist2 = point_range_to_linear_rtree
            <
                PointRange2, Geometry1, Strategy
            >::apply(points2, geometry1, strategy);

        return std::min(cdist1, cdist2);
    }
};


template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy
>
struct linear_to_linear_rtree<Geometry1, Geometry2, Strategy, false, true>
{
    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type return_type;   

    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2,
                                    Strategy const& strategy,
                                    bool check_intersection = true)
    {
        return linear_to_linear_rtree
            <
                Geometry2, Geometry1, Strategy, true, false
            >::apply(geometry2, geometry1, strategy, check_intersection);
    }    
};


template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy
>
struct linear_to_linear_rtree<Geometry1, Geometry2, Strategy, true, false>
{
    // the following works with linear geometries seen as ranges of points
    //
    // we compute the r-tree for the points of one range and then,
    // compute nearest points for the segments of the other,
    // ... and ...
    // vice versa.

    typedef typename point_type<Geometry1>::type Point1;
    typedef typename point_type<Geometry2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2,
                                    Strategy const& strategy,
                                    bool check_intersection = true)
    {
        if ( check_intersection && geometry::intersects(geometry1, geometry2) )
        {
            return return_type(0);
        }

        typedef std::vector<Point1> PointRange1;

        PointRange1 points1;
        get_points<Geometry1>::apply(geometry1, std::back_inserter(points1));

        return_type cdist1 = point_range_to_linear_rtree
            <
                PointRange1, Geometry2, Strategy
            >::apply(points1, geometry2, strategy);

        return_type cdist2 = point_range_to_linear_rtree
            <
                Geometry2, Geometry1, Strategy
            >::apply(geometry2, geometry1, strategy);

        return std::min(cdist1, cdist2);
    }
};


template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy
>
struct linear_to_linear_rtree<Geometry1, Geometry2, Strategy, false, false>
{
    // the following works with linear geometries seen as ranges of points
    //
    // we compute the r-tree for the points of one range and then,
    // compute nearest points for the segments of the other,
    // ... and ...
    // vice versa.

    typedef typename point_type<Geometry1>::type Point1;
    typedef typename point_type<Geometry2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2,
                                    Strategy const& strategy,
                                    bool check_intersection = true)
    {
        if ( check_intersection && geometry::intersects(geometry1, geometry2) )
        {
            return return_type(0);
        }

        return_type cdist1 = point_range_to_linear_rtree
            <
                Geometry1, Geometry2, Strategy
            >::apply(geometry1, geometry2, strategy);

        return_type cdist2 = point_range_to_linear_rtree
            <
                Geometry2, Geometry1, Strategy
            >::apply(geometry2, geometry1, strategy);

        return std::min(cdist1, cdist2);
    }
};



#if 0
template
<
    typename Geometry1, typename Geometry2,
    typename Strategy,
    typename Tag1, typename Tag2
>
struct closest_distance_rtree_dispatch
    : not_implemented<Geometry1, Geometry2, Strategy>
{};


template <typename LineString1, typename LineString2, typename Strategy>
struct closest_distance_rtree_dispatch
    <
        LineString1, LineString2, Strategy,
        linestring_tag, linestring_tag  
    > : linear_to_linear_rtree<LineString1, LineString2, Strategy>
{};


template <typename LineString, typename Ring, typename Strategy>
struct closest_distance_rtree_dispatch
    <
        LineString, Ring, Strategy,
        linestring_tag, ring_tag  
    > : linear_to_linear_rtree<LineString, Ring, Strategy>
{};


template <typename LineString, typename Polygon, typename Strategy>
struct closest_distance_rtree_dispatch
    <
        LineString, Polygon, Strategy,
        linestring_tag, polygon_tag  
    > : linear_to_linear_rtree<LineString, Polygon, Strategy>
{};


template <typename Ring1, typename Ring2, typename Strategy>
struct closest_distance_rtree_dispatch
    <
        Ring1, Ring2, Strategy,
        ring_tag, ring_tag  
    > : linear_to_linear_rtree<Ring1, Ring2, Strategy>
{};


template <typename Ring, typename Polygon, typename Strategy>
struct closest_distance_rtree_dispatch
    <
        Ring, Polygon, Strategy,
        ring_tag, polygon_tag  
    > : linear_to_linear_rtree<Ring, Polygon, Strategy>
{};


template <typename Polygon1, typename Polygon2, typename Strategy>
struct closest_distance_rtree_dispatch
    <
        Polygon1, Polygon2, Strategy,
        polygon_tag, polygon_tag  
    > : linear_to_linear_rtree<Polygon1, Polygon2, Strategy>
{};




template <typename Geometry1, typename Geometry2, typename Strategy>
inline typename strategy::distance::services::return_type
    <
          Strategy,
          typename point_type<Geometry1>::type,
          typename point_type<Geometry2>::type
    >::type
closest_distance_rtree(Geometry1 const& geometry1,
                       Geometry2 const& geometry2,
                       Strategy const& strategy)
{
    return closest_distance_rtree_dispatch
        <
            Geometry1,
            Geometry2,
            Strategy,
            typename tag<Geometry1>::type,
            typename tag<Geometry2>::type
        >::apply(geometry1, geometry2, strategy);
}




template <typename Geometry1, typename Geometry2>
inline typename default_distance_result<Geometry1, Geometry2>::type
closest_distance_rtree(Geometry1 const& geometry1,
                       Geometry2 const& geometry2)
{
    return closest_distance_rtree_dispatch
        <
            Geometry1,
            Geometry2,
            typename detail::distance::default_strategy<Geometry1, Geometry2>,
            typename tag<Geometry1>::type,
            typename tag<Geometry2>::type
        >::apply(geometry1, geometry2, strategy);
}
#endif




}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_HPP
