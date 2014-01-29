// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_BRUTE_FORCE_CLOSEST_PAIR_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_BRUTE_FORCE_CLOSEST_PAIR_HPP

#include <utility>
#include <boost/range.hpp>
#include <boost/geometry/algorithms/distance.hpp>

namespace boost { namespace geometry
{

namespace detail { namespace distance
{


template <typename Strategy>
struct brute_force_closest_pair
{
    template <typename Range1, typename Range2, typename Distance>
    static inline std::pair
        <
            typename boost::range_value<Range1 const>::type,
            typename boost::range_value<Range2 const>::type
        >
    apply(Range1 const& r1, Range2 const& r2,
          Strategy const& strategy,
          Distance& dmin)
    {
        typedef typename boost::range_iterator<Range1 const>::type Iterator1;
        typedef typename boost::range_iterator<Range2 const>::type Iterator2;

        typedef typename boost::range_value<Range1 const>::type Value1;
        typedef typename boost::range_value<Range2 const>::type Value2;

        BOOST_AUTO_TPL(it1, boost::begin(r1));

        dmin = geometry::distance(**it1, **boost::begin(r2), strategy);
        Value1 it1_min = *it1;
        Value2 it2_min = *boost::begin(r2);

        for (; it1 != boost::end(r1); ++it1)
        {
            BOOST_AUTO_TPL(it2, boost::begin(r2));
            for (; it2 != boost::end(r2); ++it2)
            {
                Distance d = geometry::distance(**it1, **it2, strategy);
                if ( d < dmin )
                {
                    it1_min = *it1;
                    it2_min = *it2;
                    dmin = d;
                }
            }
        }

        return std::make_pair(it1_min, it2_min);
    }


    template <typename Range, typename Distance>
    std::pair
    <
        typename boost::range_iterator<Range const>::type,
        typename boost::range_iterator<Range const>::type
    >
    static inline apply(Range const& r, Strategy const& strategy,
                        Distance& dmin)
    {
        typedef typename boost::range_iterator<Range const>::type Iterator;
        typedef typename boost::range_value<Range const>::type Value;

        BOOST_AUTO_TPL(it1, boost::begin(r));
        BOOST_AUTO_TPL(it2, boost::begin(r));
        ++it2;

        dmin = geometry::distance(*it1, *it2, strategy);
        Iterator it1_min = it1;
        Iterator it2_min = it2;

        for (; it1 != boost::end(r); ++it1)
        {
            it2 = it1;
            ++it2;
            for (; it2 != boost::end(r); ++it2)
            {
                Distance d = geometry::distance(*it1, *it2, strategy);
                if ( d < dmin )
                {
                    it1_min = it1;
                    it2_min = it2;
                    dmin = d;
                }
            }
        }

        return std::make_pair(it1_min, it2_min);
    }
};



}} // namespace detail::distance

}} // namespace boost::geometry




#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_BRUTE_FORCE_CLOSEST_PAIR_HPP
