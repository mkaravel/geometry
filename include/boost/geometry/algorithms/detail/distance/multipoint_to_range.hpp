// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_MULTIPOINT_TO_RANGE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_MULTIPOINT_TO_RANGE_HPP

#include <boost/numeric/conversion/bounds.hpp>
#include <boost/range.hpp>

#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/core/geometry_id.hpp>
#include <boost/geometry/multi/core/point_type.hpp>
#include <boost/geometry/multi/geometries/concepts/check.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/multi/algorithms/num_points.hpp>
#include <boost/geometry/util/select_coordinate_type.hpp>

#include <boost/geometry/algorithms/detail/distance/multi_to_multi.hpp>
#include <boost/geometry/algorithms/detail/distance/range_to_range.hpp>
#include <boost/geometry/algorithms/detail/distance/closest_distance_rtree.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{

template <typename MultiPoint1, typename MultiPoint2, typename Strategy>
struct multipoint_to_multipoint
{
    typedef typename point_type<MultiPoint1>::type Point1;
    typedef typename point_type<MultiPoint2>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type comparable_strategy;

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

        comparable_strategy cstrategy =
            strategy::distance::services::get_comparable
                <
                    Strategy
                >::apply(strategy);

        return_type min_cdist = range_to_range_rtree
            <
                MultiPoint1, MultiPoint2, comparable_strategy
            >::apply(multipoint1, multipoint2, cstrategy);

        return strategy::distance::services::comparable_to_regular
            <
                comparable_strategy,
                Strategy,
                MultiPoint1,
                MultiPoint2
            >::apply(min_cdist);
    }
};


template <typename MultiPoint, typename Geometry, typename Strategy>
struct multipoint_to_linear
{
    typedef typename point_type<MultiPoint>::type Point1;
    typedef typename point_type<Geometry>::type Point2;

    typedef typename strategy::distance::services::return_type
        <
            Strategy, Point1, Point2
        >::type return_type;   

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type comparable_strategy;

    static inline return_type apply(MultiPoint const& multipoint,
                                    Geometry const& geometry,
                                    Strategy const& strategy)
    {
        comparable_strategy cstrategy =
            strategy::distance::services::get_comparable
                <
                    Strategy
                >::apply(strategy);

        return_type min_cdist = point_range_to_linear_rtree
            <
                MultiPoint, Geometry, comparable_strategy
            >::apply(multipoint, geometry, cstrategy);

        return strategy::distance::services::comparable_to_regular
            <
                comparable_strategy,
                Strategy,
                MultiPoint,
                Geometry
            >::apply(min_cdist);
    }

    static inline return_type apply(Geometry const& geometry,
                                    MultiPoint const& multipoint,
                                    Strategy const& strategy)
    {
        return apply(multipoint, geometry, strategy);
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL



#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


namespace splitted_dispatch
{

// specializations of distance_multi_to_multi for various geometry combinations


// multipoint-multipoint
template <typename MultiPoint1, typename MultiPoint2, typename Strategy>
struct distance_multi_to_multi
    <
        MultiPoint1, MultiPoint2, Strategy,
        multi_point_tag, multi_point_tag
    > : detail::distance::multipoint_to_multipoint
        <
            MultiPoint1, MultiPoint2, Strategy
        >
{};


// multipoint-multilinestring
template
<
    typename MultiPoint,
    typename MultiLinestring,
    typename Strategy
>
struct distance_multi_to_multi
    <
        MultiPoint, MultiLinestring, Strategy,
        multi_point_tag, multi_linestring_tag
    > : detail::distance::multipoint_to_linear
        <
            MultiPoint, MultiLinestring, Strategy
        >
{};

} // namespace splitted_dispatch


} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH



}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DISTANCE_ALTERNATE_HPP
