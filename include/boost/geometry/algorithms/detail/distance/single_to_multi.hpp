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

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SINGLE_TO_MULTI_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SINGLE_TO_MULTI_HPP

#include <boost/numeric/conversion/bounds.hpp>
#include <boost/range.hpp>

#include <boost/geometry/multi/core/tags.hpp>
#include <boost/geometry/multi/core/geometry_id.hpp>
#include <boost/geometry/multi/core/point_type.hpp>
#include <boost/geometry/multi/geometries/concepts/check.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/multi/algorithms/num_points.hpp>
#include <boost/geometry/util/select_coordinate_type.hpp>

#include <boost/geometry/algorithms/detail/distance/range_to_range.hpp>
#include <boost/geometry/algorithms/detail/distance/closest_distance_rtree.hpp>

#include <boost/geometry/multi/multi.hpp>

namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// single-to-multi
template<typename Geometry, typename MultiGeometry, typename Strategy>
struct distance_single_to_multi
    : private dispatch::distance
      <
          Geometry,
          typename range_value<MultiGeometry>::type,
          Strategy
      >
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
                         typename point_type<Geometry>::type,
                         typename point_type<MultiGeometry>::type
                     >::type return_type;

    static inline return_type apply(Geometry const& geometry,
                                    MultiGeometry const& multi,
                                    Strategy const& strategy)
    {
        return_type min_cdist = return_type();
        bool first = true;

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        for (typename range_iterator<MultiGeometry const>::type it = boost::begin(multi);
                it != boost::end(multi);
                ++it, first = false)
        {
            return_type cdist = dispatch::distance
                <
                    Geometry,
                    typename range_value<MultiGeometry>::type,
                    ComparableStrategy
                >::apply(geometry, *it, cstrategy);

            if (first || cdist < min_cdist)
            {
                min_cdist = cdist;
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Geometry,
                MultiGeometry
            >::apply(min_cdist);
    }


    static inline return_type apply(MultiGeometry const& multi,
                                    Geometry const& geometry,
                                    Strategy const& strategy)
    {
        return apply(geometry, multi, strategy);
    }
};


// multi-to-multi
template<typename Multi1, typename Multi2, typename Strategy>
struct distance_multi_to_multi
    : private distance_single_to_multi
      <
          typename range_value<Multi1>::type,
          Multi2,
          Strategy
      >
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
            return_type cdist = distance_single_to_multi
                <
                    typename range_value<Multi1>::type,
                    Multi2,
                    ComparableStrategy
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


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_SINGLE_TO_MULTI_HPP
