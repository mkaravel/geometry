// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_BOX_TO_BOX_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_BOX_TO_BOX_HPP

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/distance_comparable_to_regular.hpp>

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/algorithms/detail/distance/point_to_box.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


template
<
    typename Box1,
    typename Box2,
    typename Strategy
>
class distance_box_box_2D
{
private:
    typedef typename point_type<Box1>::type BoxPoint1;
    typedef typename point_type<Box2>::type BoxPoint2;

    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    typedef typename strategy::distance::services::tag
        <
            ComparableStrategy
        >::type ComparableStrategyTag;

    typedef dispatch::distance
        <
            BoxPoint1, Box2, ComparableStrategy,
            point_tag, box_tag, ComparableStrategyTag, false
        > PointToBox12;

    typedef dispatch::distance
        <
            BoxPoint2, Box1, ComparableStrategy,
            point_tag, box_tag, ComparableStrategyTag, false
        > PointToBox21;


public:
    typedef typename strategy::distance::services::return_type
        <
            Strategy, BoxPoint1, BoxPoint2
        >::type return_type;


    static inline return_type apply(Box1 const& box1,
                                    Box2 const& box2,
                                    Strategy const& strategy)
    {
        if ( geometry::intersects(box1, box2) )
        {
            return 0;
        }

        return_type d[8];

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        BoxPoint1 top_left1, bottom_right1;
        geometry::assign_values(top_left1,
                                geometry::get<0>(box1.min_corner()),
                                geometry::get<1>(box1.max_corner()));

        geometry::assign_values(bottom_right1,
                                geometry::get<0>(box1.max_corner()),
                                geometry::get<1>(box1.min_corner()));

        BoxPoint2 top_left2, bottom_right2;
        geometry::assign_values(top_left2,
                                geometry::get<0>(box2.min_corner()),
                                geometry::get<1>(box2.max_corner()));

        geometry::assign_values(bottom_right2,
                                geometry::get<0>(box2.max_corner()),
                                geometry::get<1>(box2.min_corner()));
        
        d[0] = PointToBox12::apply(box1.min_corner(), box2, cstrategy);
        d[1] = PointToBox12::apply(box1.max_corner(), box2, cstrategy);
        d[2] = PointToBox12::apply(top_left1, box2, cstrategy);
        d[3] = PointToBox12::apply(bottom_right1, box2, cstrategy);

        d[4] = PointToBox21::apply(box2.min_corner(), box1, cstrategy);
        d[5] = PointToBox21::apply(box2.max_corner(), box1, cstrategy);
        d[6] = PointToBox21::apply(top_left2, box1, cstrategy);
        d[7] = PointToBox21::apply(bottom_right2, box1, cstrategy);

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Box1,
                Box2
            >::apply( *std::min_element(d, d+8) );
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

namespace splitted_dispatch
{


template
<
    typename Box1,
    typename Box2,
    std::size_t BoxDimension1,
    std::size_t BoxDimension2,
    typename Strategy
>
struct distance_box_to_box
    : not_implemented<Box1, Box2, Strategy>
{};


template
<
    typename Box1,
    typename Box2,
    typename Strategy
>
struct distance_box_to_box<Box1, Box2, 2, 2, Strategy>
    : detail::distance::distance_box_box_2D<Box1, Box2, Strategy>
{};


} // namespace splitted_dispatch

// segment-box
template <typename Box1, typename Box2, typename Strategy>
struct distance
    <
        Box1, Box2, Strategy, box_tag, box_tag,
        strategy_tag_distance_point_segment, false
    >
    : splitted_dispatch::distance_box_to_box
        <
            Box1, Box2,
            dimension<Box1>::type::value,
            dimension<Box2>::type::value,
            Strategy
        >
{};



} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_BOX_TO_BOX_HPP
