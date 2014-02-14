#ifndef BOOST_GEOMETRY_STRATGIES_CARTESIAN_DISTANCE_COMPARABLE_TO_REGULAR_HPP
#define BOOST_GEOMETRY_STRATGIES_CARTESIAN_DISTANCE_COMPARABLE_TO_REGULAR_HPP


namespace boost { namespace geometry
{

namespace strategy { namespace distance
{



#ifndef DOXYGEN_NO_STRATEGY_SPECIALIZATIONS
namespace services
{


template
<
    typename ComparableStrategy,
    typename Strategy,
    typename Geometry1,
    typename Geometry2
>
struct comparable_to_regular
{
    typedef typename return_type
        <
            Strategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type calculation_type;

    typedef typename return_type
        <
            ComparableStrategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type comparable_calculation_type;

    static inline calculation_type apply(comparable_calculation_type const& cd)
    {
        return sqrt( boost::numeric_cast<calculation_type>(cd) );
    }
};



template <typename ComparableStrategy, typename Geometry1, typename Geometry2>
struct comparable_to_regular
    <
        ComparableStrategy,
        ComparableStrategy,
        Geometry1,
        Geometry2
    >
{
    typedef typename return_type
        <
            ComparableStrategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type comparable_calculation_type;

    static inline comparable_calculation_type
    apply(comparable_calculation_type const& cd)
    {
        return cd;
    }
};







} // namespace services
#endif // DOXYGEN_NO_STRATEGY_SPECIALIZATIONS


}} // namespace strategy::distance

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_STRATGIES_CARTESIAN_DISTANCE_COMPARABLE_TO_REGULAR_HPP
