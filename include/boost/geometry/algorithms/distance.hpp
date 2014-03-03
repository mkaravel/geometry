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

#ifndef BOOST_GEOMETRY_ALGORITHMS_DISTANCE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DISTANCE_HPP

#include <boost/concept_check.hpp>
#include <boost/mpl/if.hpp>
#include <boost/typeof/typeof.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/reverse_dispatch.hpp>
#include <boost/geometry/core/tag_cast.hpp>

#include <boost/geometry/algorithms/not_implemented.hpp>
#include <boost/geometry/algorithms/detail/throw_on_empty_input.hpp>

#include <boost/geometry/geometries/concepts/check.hpp>

#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/default_distance_result.hpp>

#include <boost/geometry/algorithms/detail/distance/default_strategies.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


using strategy::distance::services::return_type;


template
<
    typename Geometry1, typename Geometry2,
    typename Strategy = typename detail::distance::default_strategy<Geometry1, Geometry2>::type,
    typename Tag1 = typename tag<Geometry1>::type,
    typename Tag2 = typename tag<Geometry2>::type,
    typename StrategyTag = typename strategy::distance::services::tag<Strategy>::type,
    typename TypeTag1 = typename tag_cast
    <
        typename tag<Geometry1>::type,
        pointlike_tag,
        linear_tag,
        areal_tag
    >::type,
    typename TypeTag2 = typename tag_cast
    <
        typename tag<Geometry2>::type,
        pointlike_tag,
        linear_tag,
        areal_tag
    >::type,
    typename SingleMultiTag1 = typename tag_cast
    <
        typename tag<Geometry1>::type,
        single_tag,
        multi_tag
    >::type,
    typename SingleMultiTag2 = typename tag_cast
    <
        typename tag<Geometry2>::type,
        single_tag,
        multi_tag
    >::type,    
    bool SegmentOrBox1 = boost::is_same<Tag1, segment_tag>::value
    || boost::is_same<Tag1, box_tag>::value,
    bool SegmentOrBox2 = boost::is_same<Tag2, segment_tag>::value
    || boost::is_same<Tag2, box_tag>::value,
    bool Reverse = reverse_dispatch<Geometry1, Geometry2>::type::value
>
struct distance: not_implemented<Tag1, Tag2>
{};



// If reversal is needed, perform it
template
<
    typename Geometry1, typename Geometry2, typename Strategy,
    typename Tag1, typename Tag2, typename StrategyTag,
    typename TypeTag1, typename TypeTag2,
    typename SingleMultiTag1, typename SingleMultiTag2,
    bool SegmentOrBox1, bool SegmentOrBox2
>
struct distance
<
    Geometry1, Geometry2, Strategy,
    Tag1, Tag2, StrategyTag,
    TypeTag1, TypeTag2,
    SingleMultiTag1, SingleMultiTag2,
    SegmentOrBox1, SegmentOrBox2,
    true
>
{
    typedef typename strategy::distance::services::return_type
                     <
                         Strategy,
                         typename point_type<Geometry2>::type,
                         typename point_type<Geometry1>::type
                     >::type return_type;

    static inline return_type apply(Geometry1 const& g1,
                                    Geometry2 const& g2,
                                    Strategy const& strategy)
    {
        return distance
            <
                Geometry2, Geometry1, Strategy,
                Tag2, Tag1, StrategyTag,
                TypeTag2, TypeTag1,
                SingleMultiTag2, SingleMultiTag1,
                SegmentOrBox2, SegmentOrBox1,
                false
            >::apply(g2, g1, strategy);
    }
};






} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH

/*!
\brief \brief_calc2{distance} \brief_strategy
\ingroup distance
\details
\details \details_calc{area}. \brief_strategy. \details_strategy_reasons

\tparam Geometry1 \tparam_geometry
\tparam Geometry2 \tparam_geometry
\tparam Strategy \tparam_strategy{Distance}
\param geometry1 \param_geometry
\param geometry2 \param_geometry
\param strategy \param_strategy{distance}
\return \return_calc{distance}
\note The strategy can be a point-point strategy. In case of distance point-line/point-polygon
    it may also be a point-segment strategy.

\qbk{distinguish,with strategy}

\qbk{
[heading Available Strategies]
\* [link geometry.reference.strategies.strategy_distance_pythagoras Pythagoras (cartesian)]
\* [link geometry.reference.strategies.strategy_distance_haversine Haversine (spherical)]
\* [link geometry.reference.strategies.strategy_distance_cross_track Cross track (spherical\, point-to-segment)]
\* [link geometry.reference.strategies.strategy_distance_projected_point Projected point (cartesian\, point-to-segment)]
\* more (currently extensions): Vincenty\, Andoyer (geographic)
}
 */

/*
Note, in case of a Compilation Error:
if you get:
 - "Failed to specialize function template ..."
 - "error: no matching function for call to ..."
for distance, it is probably so that there is no specialization
for return_type<...> for your strategy.
*/
template <typename Geometry1, typename Geometry2, typename Strategy>
inline typename strategy::distance::services::return_type
                <
                    Strategy,
                    typename point_type<Geometry1>::type,
                    typename point_type<Geometry2>::type
                >::type
distance(Geometry1 const& geometry1,
         Geometry2 const& geometry2,
         Strategy const& strategy)
{
    concept::check<Geometry1 const>();
    concept::check<Geometry2 const>();

    detail::throw_on_empty_input(geometry1);
    detail::throw_on_empty_input(geometry2);

    return dispatch::distance
               <
                   Geometry1,
                   Geometry2,
                   Strategy
               >::apply(geometry1, geometry2, strategy);
}


/*!
\brief \brief_calc2{distance}
\ingroup distance
\details The default strategy is used, corresponding to the coordinate system of the geometries
\tparam Geometry1 \tparam_geometry
\tparam Geometry2 \tparam_geometry
\param geometry1 \param_geometry
\param geometry2 \param_geometry
\return \return_calc{distance}

\qbk{[include reference/algorithms/distance.qbk]}
 */
template <typename Geometry1, typename Geometry2>
inline typename default_distance_result<Geometry1, Geometry2>::type distance(
                Geometry1 const& geometry1, Geometry2 const& geometry2)
{
    concept::check<Geometry1 const>();
    concept::check<Geometry2 const>();

    return distance(geometry1, geometry2,
                    typename detail::distance::default_strategy<Geometry1, Geometry2>::type());
}

}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DISTANCE_HPP
