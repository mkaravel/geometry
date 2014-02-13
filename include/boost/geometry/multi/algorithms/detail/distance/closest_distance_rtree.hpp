// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_HPP

#include <boost/foreach.hpp>
#include <boost/range.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/algorithms/envelope.hpp>

#include <boost/geometry/multi/algorithms/detail/distance/split_to_segments.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/get_points.hpp>

#include <cstdlib>
#include <cassert>
#include <limits>

namespace boost { namespace geometry
{


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
#if 1
        // create -- packing algorithm
        index::rtree
            <
                RTreeValue, index::linear<8>
            > rt(boost::begin(t_range), boost::end(t_range));
#else
        // or balancing algorithm
        index::rtree<RTeeeValue, index::linear<8> > rt;
        BOOST_FOREACH(RTreeValue const& rt_v, t_range)
            rt.insert(rt_v);
#endif


        BOOST_AUTO_TPL(it, boost::begin(q_range));

        QueryValue q_v(*it);

        RTreeValue t_v;
        std::size_t n = rt.query(index::nearest(q_v, 1), &t_v);
        assert( n > 0 );
        return_type min_cd = geometry::comparable_distance(t_v, q_v, strategy);

        ++it;
        for (; it != boost::end(q_range); ++it)
        {
            q_v = *it;
            n = rt.query(index::nearest(q_v, 1), &t_v);
            assert( n > 0 );
            return_type cd = geometry::comparable_distance(t_v, q_v, strategy);
            if ( cd < min_cd )
            {
                min_cd = cd;
            }
        }

        return min_cd;
    }
};



template <typename TreeRange, typename QueryRange, typename Strategy>
struct range_to_range_rtree2
{
    typedef typename boost::range_value<TreeRange>::type TreeRangeValue;
    typedef typename boost::range_value<QueryRange>::type QueryRangeValue;

    typedef typename point_type<TreeRangeValue>::type Point1;
    typedef typename point_type<QueryRangeValue>::type Point2;

    typedef geometry::model::box<Point1> Box1;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   

    static inline return_type apply(TreeRange const& t_range,
                                    QueryRange const& q_range,
                                    Strategy const& strategy)
    {
        // compute the bounding boxes of the segments
        typedef std::pair<Box1,TreeRangeValue> RTreeValue;

        std::vector<RTreeValue> boxes_and_segs;
        BOOST_FOREACH(TreeRangeValue const& seg, t_range)
        {
            Box1 box;
            geometry::envelope(seg, box);          
            boxes_and_segs.push_back( std::make_pair(box, seg) );
        }

        typedef index::rtree
            <
                RTreeValue, index::linear<8>
            > RTree;

#if 1
        // create -- packing algorithm
        RTree rt(boost::begin(boxes_and_segs), boost::end(boxes_and_segs));
#else
        // or balancing algorithm
        RTree rt;
        BOOST_FOREACH(RTreeValue const& rt_v, boxes_and_segs)
            rt.insert(rt_v);
#endif

        BOOST_AUTO_TPL(it, boost::begin(q_range));

        QueryRangeValue q_v(*it);

        // initialize distance somehow
        RTreeValue t_v;
        std::size_t n = rt.query(index::nearest(q_v, 1), &t_v);
        assert( n > 0 );
        return_type min_cd = geometry::comparable_distance(t_v.second,
                                                           q_v,
                                                           strategy);

        // now go over all segments in query range and find smallest distances
        for (; it != boost::end(q_range); ++it)
        {
            q_v = *it;
            typename RTree::const_query_iterator qit
                = rt.qbegin( index::nearest(q_v, rt.size()) );

            for (; qit != rt.qend(); ++qit)
            {
                // distance to box
                return_type cd = geometry::comparable_distance(qit->first,
                                                               q_v,
                                                               strategy);
                if ( cd >= min_cd )
                {
                    break;
                }

                // actual distance to segment
                cd = geometry::comparable_distance(qit->second, q_v, strategy);
                if ( cd < min_cd )
                {
                    min_cd = cd;
                }
                if ( min_cd == 0 )
                {
                    return 0;
                }
            }
        }

        return min_cd;
    }
};



template <typename MultiPoint1, typename MultiPoint2, typename Strategy>
struct multipoint_to_multipoint
{
    typedef typename point_type<MultiPoint1>::type Point1;
    typedef typename point_type<MultiPoint2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   


    static inline return_type apply(MultiPoint1 const& multipoint1,
                                    MultiPoint2 const& multipoint2,
                                    Strategy const& strategy)
    {
        if ( boost::size(multipoint1) > boost::size(multipoint2) )

        {
            return multipoint_to_multipoint
                <
                    MultiPoint2, MultiPoint1, Strategy
                >::apply(multipoint2, multipoint1, strategy);
        }

        return_type min_cdist = range_to_range_rtree
            <
                MultiPoint1, MultiPoint2, Strategy
            >::apply(multipoint1, multipoint2, strategy);
        return std::sqrt(min_cdist);
    }
};


template <typename Geometry1, typename Geometry2, typename Strategy>
struct linear_to_linear
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
            return 0;
        }

        typedef geometry::model::segment<Point1> Segment1;
        typedef geometry::model::segment<Point2> Segment2;

        typedef std::vector<Point1> PointRange1;
        typedef std::vector<Point2> PointRange2;
        PointRange1 points1;
        PointRange2 points2;

        get_points<Geometry1>::apply(geometry1, std::back_inserter(points1));
        get_points<Geometry2>::apply(geometry2, std::back_inserter(points2));

        typedef std::vector<Segment1> SegmentRange1;
        typedef std::vector<Segment2> SegmentRange2;
        SegmentRange1 segments1;
        SegmentRange2 segments2;

        split_to_segments<Geometry1>::apply(geometry1,
                                            std::back_inserter(segments1));
        split_to_segments<Geometry2>::apply(geometry2,
                                            std::back_inserter(segments2));

        return_type cdist1 = range_to_range_rtree
            <
                PointRange1, SegmentRange2, Strategy
            >::apply(points1, segments2, strategy);

        return_type cdist2 = range_to_range_rtree
            <
                PointRange2, SegmentRange1, Strategy
            >::apply(points2, segments1, strategy);

        return std::sqrt( std::min(cdist1, cdist2) );
    }
};


template <typename Geometry1, typename Geometry2, typename Strategy>
struct linear_to_linear2
{
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
        typedef geometry::model::segment<Point1> Segment1;
        typedef geometry::model::segment<Point2> Segment2;

        typedef std::vector<Segment1> SegmentRange1;
        typedef std::vector<Segment2> SegmentRange2;
        SegmentRange1 segments1;
        SegmentRange2 segments2;

        split_to_segments<Geometry1>::apply(geometry1,
                                            std::back_inserter(segments1));
        split_to_segments<Geometry2>::apply(geometry2,
                                            std::back_inserter(segments2));

        if ( boost::size(segments1) < boost::size(segments2) )
        {
            return_type cdist = range_to_range_rtree2
                <
                    SegmentRange2, SegmentRange1, Strategy
                >::apply(segments2, segments1, strategy);
            return std::sqrt( cdist );
        }

        return_type cdist = range_to_range_rtree2
            <
                SegmentRange1, SegmentRange2, Strategy
            >::apply(segments1, segments2, strategy);

        return std::sqrt( cdist );
    }
};





template
<
    typename Geometry1, typename Geometry2,
    typename Strategy,
    typename Tag1, typename Tag2
>
struct closest_distance_rtree_dispatch
    : not_implemented<Geometry1, Geometry2, Strategy>
{};


template
<
    typename MultiPoint1, typename MultiPoint2, typename Strategy
>
struct closest_distance_rtree_dispatch
    <
        MultiPoint1, MultiPoint2, Strategy,
        multi_point_tag, multi_point_tag
    > : multipoint_to_multipoint<MultiPoint1, MultiPoint2, Strategy>
{};


template
<
    typename MultiLineString1, typename MultiLineString2, typename Strategy
>
struct closest_distance_rtree_dispatch
    <
        MultiLineString1, MultiLineString2, Strategy,
        multi_linestring_tag, multi_linestring_tag
    > : linear_to_linear<MultiLineString1, MultiLineString2, Strategy>
{};






template
<
    typename Geometry1,
    typename Geometry2,
    typename PointToPointStrategy = typename detail::distance::default_strategy
    <
        typename point_type<Geometry1>::type,
        typename point_type<Geometry2>::type
    >::type
>
struct closest_distance_rtree
{
    typedef PointToPointStrategy Strategy;

    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type return_type;   


    static inline return_type apply(Geometry1 const& geometry1,
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


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2)
    {
        return closest_distance_rtree_dispatch
            <
                Geometry1,
                Geometry2,
                Strategy,
                typename tag<Geometry1>::type,
                typename tag<Geometry2>::type
            >::apply(geometry1, geometry2, Strategy());
    }
};





template
<
    typename Geometry1,
    typename Geometry2,
    typename PointToPointStrategy = typename detail::distance::default_strategy
    <
        typename point_type<Geometry1>::type,
        typename point_type<Geometry2>::type
    >::type
>
struct closest_distance_rtree_linear
{
    typedef PointToPointStrategy Strategy;

    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type return_type;   


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2,
                                    Strategy const& strategy)
    {
        return linear_to_linear2
            <
                Geometry1,
                Geometry2,
                Strategy
            >::apply(geometry1, geometry2, strategy);
    }


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2)
    {
        return linear_to_linear2
            <
                Geometry1,
                Geometry2,
                Strategy
            >::apply(geometry1, geometry2, Strategy());
    }
};



}} // namespace detail::distance

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_HPP
