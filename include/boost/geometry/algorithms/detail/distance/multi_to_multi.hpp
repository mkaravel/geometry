// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2014 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2014 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2014 Mateusz Loskot, London, UK.

// This file was modified by Oracle on 2014.
// Modifications copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_MULTI_TO_MULTI_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_MULTI_TO_MULTI_HPP


#include <boost/geometry/algorithms/detail/distance/single_to_multi.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// generic multi-to-multi
template
<
    typename Multi1,
    typename Multi2,
    typename Strategy,
    typename Tag1,
    typename Tag2
>
struct distance_multi_to_multi_generic
{
    typedef typename strategy::distance::services::comparable_type
        <
            Strategy
        >::type ComparableStrategy;

    typedef typename strategy::distance::services::get_comparable
        <
            Strategy
        > GetComparable;

    typedef typename strategy::distance::services::return_type
                     <
                         Strategy,
                         typename point_type<Multi1>::type,
                         typename point_type<Multi2>::type
                     >::type return_type;

    static inline return_type apply(Multi1 const& multi1,
                Multi2 const& multi2, Strategy const& strategy)
    {
        return_type min_cdist = return_type();
        bool first = true;

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        for(typename range_iterator<Multi1 const>::type it = boost::begin(multi1);
                it != boost::end(multi1);
                ++it, first = false)
        {
            return_type cdist = dispatch::distance_single_to_multi
                <
                    typename range_value<Multi1>::type,
                    Multi2,
                    ComparableStrategy,
                    typename tag<typename range_value<Multi1>::type>::type,
                    typename tag<Multi2>::type
                >::apply(*it, multi2, cstrategy);
            if (first || cdist < min_cdist)
            {
                min_cdist = cdist;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Multi1,
                Multi2
            >::apply(min_cdist);
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL



#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{



// default sub-dispatch for multi-multi geometries
template
<
    typename MultiGeometry1,
    typename MultiGeometry2,
    typename Strategy,
    typename Tag1,
    typename Tag2
>
struct distance_multi_to_multi
    : detail::distance::distance_multi_to_multi_generic
        <
            MultiGeometry1, MultiGeometry2, Strategy, Tag1, Tag2
        >
{};




// dispatch for multi-multi geometries
template
<
    typename MultiGeometry1,
    typename MultiGeometry2,
    typename Strategy
>
struct distance
    <
        MultiGeometry1, MultiGeometry2, Strategy, multi_tag, multi_tag,
        strategy_tag_distance_point_segment, false
    > : distance_multi_to_multi
        <
            MultiGeometry1, MultiGeometry2, Strategy,
            typename geometry::tag<MultiGeometry1>::type,
            typename geometry::tag<MultiGeometry2>::type
        >
{};






} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_MULTI_TO_MULTI_HPP
