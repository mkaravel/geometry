// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_EXTREMAL_COORDINATE_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_EXTREMAL_COORDINATE_HPP

namespace boost { namespace geometry
{

namespace detail { namespace distance
{

// minimum coordinate for a range
template <typename Range, std::size_t Dimension, typename Less>
struct extremal_coordinate_range
{
    template <typename Point>
    struct coordinate_less
    {
        Less const& m_less;

        coordinate_less(Less const& less) : m_less(less) {}

        bool operator()(Point const& p, Point const& q) const
        {
            return m_less(geometry::get<Dimension>(p),
                          geometry::get<Dimension>(q));
        }
    };

    static inline typename coordinate_type<Range>::type
    apply(Range const& r, Less const& less)
    {
        typedef typename point_type<Range>::type Point;

        return geometry::get<Dimension>
            (*std::min_element(
                               boost::begin(r),
                               boost::end(r),
                               coordinate_less<Point>(less)
                               )
             );
    }
};


// extremal coordinate for a range of ranges (multi-range)
template <typename MultiGeometry, std::size_t Dimension, typename Less>
struct extremal_coordinate_multi_range
{
    static inline typename coordinate_type<MultiGeometry>::type
    apply(MultiGeometry const& multigeometry, Less const& less)
    {
        typedef typename coordinate_type<MultiGeometry>::type return_type;
        typedef typename boost::range_value<MultiGeometry>::type Geometry;
        typedef extremal_coordinate_range<Geometry, Dimension, Less> MCR;

        BOOST_AUTO_TPL(it, boost::begin(multigeometry));

        return_type extremal_coord = MCR::apply(*it, less);
        if ( it != boost::end(multigeometry) )
        {
            for (++it; it != boost::end(multigeometry); ++it)
            {
                return_type coord = MCR::apply(*it, less);
                if ( less(coord, extremal_coord) )
                {
                    extremal_coord = coord;
                }                                     
            }
        }

        return extremal_coord;
    }

};




// dispatching
template
<
    typename Geometry, std::size_t Dimension,
    typename Less,typename GeometryTag
>
struct extremal_coordinate_dispatch
    : not_implemented<Geometry>
{};

// point
template <typename Point, std::size_t Dimension, typename Less>
struct extremal_coordinate_dispatch<Point, Dimension, Less, point_tag>
    : extremal_coordinate_range<Point, Dimension, Less>
{
    static inline typename coordinate_type<Point>::type
    apply(Point const& point, Less const&)
    {
        return geometry::get<Dimension>(point);
    }
};

// linestring
template <typename LineString, std::size_t Dimension, typename Less>
struct extremal_coordinate_dispatch<LineString, Dimension, Less, linestring_tag>
    : extremal_coordinate_range<LineString, Dimension, Less>
{};

// polygon
template <typename Polygon, std::size_t Dimension, typename Less>
struct extremal_coordinate_dispatch<Polygon, Dimension, Less, polygon_tag>
{
    static inline typename coordinate_type<Polygon>::type
    apply(Polygon const& polygon, Less const& less)
    {
        return extremal_coordinate_range
            <
                typename ring_type<Polygon>::type,
                Dimension,
                Less
            >::apply(exterior_ring(polygon), less);
    }
};

// multipoint
template <typename MultiPoint, std::size_t Dimension, typename Less>
struct extremal_coordinate_dispatch
    <
        MultiPoint, Dimension, Less, multi_point_tag
    > : extremal_coordinate_range<MultiPoint, Dimension, Less>
{};

// multilinestring
template <typename MultiLineString, std::size_t Dimension, typename Less>
struct extremal_coordinate_dispatch
    <
        MultiLineString, Dimension, Less, multi_linestring_tag
    > : extremal_coordinate_multi_range<MultiLineString, Dimension, Less>
{};

// multipolygon
template <typename MultiPolygon, std::size_t Dimension, typename Less>
struct extremal_coordinate_dispatch
    <
        MultiPolygon, Dimension, Less, multi_polygon_tag
    > : extremal_coordinate_multi_range<MultiPolygon, Dimension, Less>
{};





// extremal coordinate function
template
<
    std::size_t Dimension,
    typename Geometry,
    typename Less
>
typename coordinate_type<Geometry>::type
extremal_coordinate(Geometry const& geometry, Less const& less)
{
    return extremal_coordinate_dispatch
        <
            Geometry,
            Dimension,
            Less,
            typename geometry::tag<Geometry>::type
        >::apply(geometry, less);
}


template
<
    std::size_t Dimension,
    typename Geometry
>
typename coordinate_type<Geometry>::type
extremal_coordinate(Geometry const& geometry)
{
    typedef typename std::less<typename coordinate_type<Geometry>::type> Less;

    return extremal_coordinate_dispatch
        <
            Geometry,
            Dimension,
            Less,
            typename geometry::tag<Geometry>::type
        >::apply(geometry, Less());
}



}} // namespace detail::distance

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_EXTREMAL_COORDINATE_HPP
