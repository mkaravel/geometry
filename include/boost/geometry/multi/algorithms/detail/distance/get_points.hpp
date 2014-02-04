#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_GET_POINTS_HPP

#include <algorithm>


namespace boost { namespace geometry
{

namespace detail { namespace distance
{


template <typename LineString>
struct linestring_to_points
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(LineString const& ls, OutputIterator oit)
    {
        return std::copy(boost::begin(ls), boost::end(ls), oit);
    }
};


template <typename MultiLineString>
struct multilinestring_to_points
{
    template <typename OutputIterator>
    static inline
    OutputIterator apply(MultiLineString const& mls, OutputIterator oit)
    {
        BOOST_AUTO_TPL(it, boost::begin(mls));
        for (; it != boost::end(mls); ++it)
        {
            oit = linestring_to_points
                <
                    typename boost::range_value<MultiLineString>::type
                >::apply(*it, oit);
        }

        return oit;
    }
};






template <typename Geometry, typename GeometryTag>
struct get_points_dispatch
    : public not_implemented<Geometry>
{};


template <typename LineString>
struct get_points_dispatch<LineString, linestring_tag>
    : linestring_to_points<LineString>
{};


template <typename MultiLineString>
struct get_points_dispatch<MultiLineString, multi_linestring_tag>
    : multilinestring_to_points<MultiLineString>
{};








template <typename Geometry>
struct get_points
    : get_points_dispatch<Geometry, typename tag<Geometry>::type>
{};




}} // namespace detail::distance

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_SPLIT_TO_SEGMENTS_HPP
