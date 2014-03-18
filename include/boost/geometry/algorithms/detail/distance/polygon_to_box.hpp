// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POLYGON_TO_BOX_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POLYGON_TO_BOX_HPP


#include <boost/geometry/algorithms/detail/distance/polygon_to_segment.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace distance
{


// compute polygon-box distance
template <typename Polygon, typename Box, typename Strategy>
class polygon_to_box
{
private:
    typedef typename point_type<Box>::type BoxPoint;
    typedef typename point_type<Polygon>::type PolygonPoint;

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
            BoxPoint, Polygon, ComparableStrategy,
            point_tag, polygon_tag, ComparableStrategyTag, false
        > PointToPolygon;

    typedef dispatch::distance
        <
            PolygonPoint, Box, ComparableStrategy,
            point_tag, box_tag, ComparableStrategyTag, false
        > PointToBox;

public:
    typedef typename return_type
        <
            Strategy, PolygonPoint, BoxPoint
        >::type return_type;


    static inline return_type
    apply(Polygon const& polygon, Box const& box, Strategy const& strategy)
    {
        if ( geometry::intersects(polygon, box) )
        {
            return 0;
        }

        ComparableStrategy cstrategy = GetComparable::apply(strategy);

        BoxPoint top_left, bottom_right;

        geometry::assign_values(top_left,
                                geometry::get<0>(box.min_corner()),
                                geometry::get<1>(box.max_corner()));

        geometry::assign_values(bottom_right,
                                geometry::get<0>(box.max_corner()),
                                geometry::get<1>(box.min_corner()));

        return_type d[4];
        d[0] = PointToPolygon::apply(box.min_corner(), polygon, cstrategy);
        d[1] = PointToPolygon::apply(box.max_corner(), polygon, cstrategy);
        d[2] = PointToPolygon::apply(top_left, polygon, cstrategy);
        d[3] = PointToPolygon::apply(bottom_right, polygon, cstrategy);

        return_type cd_min = *std::min_element(d, d+4);

        BOOST_AUTO_TPL(eit, boost::begin( exterior_ring(polygon) ));
        for (; eit != boost::end( exterior_ring(polygon) ); ++eit)
        {
            return_type cd = PointToBox::apply(*eit, box, cstrategy);
            if ( cd < cd_min )
            {
                cd_min = cd;
            }
        }

        typename interior_return_type<Polygon const>::type rings
            = interior_rings(polygon);

        for (BOOST_AUTO_TPL(iit, boost::begin(rings)); iit != boost::end(rings); ++iit)
        {
            for (BOOST_AUTO_TPL(it, boost::begin(*iit)); it != boost::end(*iit); ++it)
            {
                return_type cd = PointToBox::apply(*it, box, cstrategy);
                if ( cd < cd_min )
                {
                    cd_min = cd;
                }
            }
        }

        return strategy::distance::services::comparable_to_regular
            <
                ComparableStrategy,
                Strategy,
                Polygon,
                Box
            >::apply( cd_min );
    }
};


}} // namespace detail::distance
#endif // DOXYGEN_NO_DETAIL



#ifndef DOXYGEN_NO_DETAIL
namespace dispatch
{


// polygon-box
template <typename Polygon, typename Box, typename Strategy>
struct distance
    <
        Polygon, Box, Strategy, polygon_tag, box_tag,
        strategy_tag_distance_point_segment, false
    >    
    : detail::distance::polygon_to_box<Polygon, Box, Strategy>
{};



} // namespace dispatch
#endif // DOXYGEN_NO_DETAIL

}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISTANCE_POLYGON_TO_BOX_HPP
