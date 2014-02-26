// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_QUERY_IT_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREE_QUERY_IT_HPP

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

#include <boost/geometry/algorithms/detail/distance/closest_distance_rtree.hpp>

namespace boost { namespace geometry
{


namespace detail { namespace distance
{



template <typename TreeRange, typename QueryRange, typename Strategy>
struct range_to_range_rtree_query_iterator
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
                if ( geometry::math::equals(min_cd, 0) )
                {
                    return 0;
                }
            }
        }

        return min_cd;
    }
};



template <typename Geometry1, typename Geometry2, typename Strategy>
struct linear_to_linear_rtree_query_iterator
{
    typedef typename point_type<Geometry1>::type Point1;
    typedef typename point_type<Geometry2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;


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

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        split_to_segments<Geometry1>::apply(geometry1,
                                            std::back_inserter(segments1));
        split_to_segments<Geometry2>::apply(geometry2,
                                            std::back_inserter(segments2));

        if ( boost::size(segments1) < boost::size(segments2) )
        {
            return_type cdist = range_to_range_rtree_query_iterator
                <
                    SegmentRange2, SegmentRange1, ComparableStrategy
                >::apply(segments2, segments1, cstrategy);

            return strategy::distance::services::comparable_to_regular
                <
                    ComparableStrategy,
                    Strategy,
                    Geometry1,
                    Geometry2
                >::apply( cdist );
        }

        return_type cdist = range_to_range_rtree_query_iterator
            <
                SegmentRange1, SegmentRange2, ComparableStrategy
            >::apply(segments1, segments2, cstrategy);

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Geometry1,
                Geometry2
            >::apply( cdist );
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
struct closest_distance_rtree_query_iterator
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
        return linear_to_linear_rtree_query_iterator
            <
                Geometry1,
                Geometry2,
                Strategy
            >::apply(geometry1, geometry2, strategy);
    }


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2)
    {
        return linear_to_linear_rtree_query_iterator
            <
                Geometry1,
                Geometry2,
                Strategy
            >::apply(geometry1, geometry2, Strategy());
    }
};



}} // namespace detail::distance

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_RTREEQUERY_IT_HPP
