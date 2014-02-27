// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_PAIR_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_PAIR_HPP

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/multi/algorithms/distance.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/random_integer_generator.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/brute_force_closest_pair.hpp>

#include <cmath>
#include <set>
#include <vector>
#include <boost/unordered_map.hpp>


namespace boost { namespace geometry
{

namespace detail { namespace distance
{



template
<
    typename MultiPoint1,
    typename MultiPoint2,
    typename PointToPointStrategy = typename detail::distance::default_strategy
    <
        typename point_type<MultiPoint1>::type,
        typename point_type<MultiPoint2>::type
    >::type
>
struct closest_pair
{
    template <typename Pair>
    struct pair_less
    { 
        pair_less() {}
        pair_less(Pair const&) {}

        inline bool operator()(Pair const& pair1, Pair const& pair2) const
        {
            if ( pair1.first != pair2.first )
            {
                return pair1.first < pair2.first;
            }
            return pair1.second < pair2.second;
        }
    };


    typedef PointToPointStrategy Strategy;
    typedef std::size_t size_type;
    typedef std::pair<size_type,size_type> Bin_id;
    typedef pair_less<Bin_id> Bin_id_less;

    template <typename Iterator>
    struct Bin {
        Bin_id id;
        std::vector<Iterator> mp;
    };


    typedef typename boost::range_iterator
        <
            MultiPoint1 const
        >::type Iterator1;

    typedef typename boost::range_iterator
        <
            MultiPoint2 const
        >::type Iterator2;

    typedef std::pair<Iterator1, Iterator2> return_type;

    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<MultiPoint1>::type,
            typename point_type<MultiPoint2>::type
        >::type distance_type;

    template <typename Range1, typename Range2, typename Distance>
    static inline std::pair
        <
            typename boost::range_iterator<Range1 const>::type,
            typename boost::range_iterator<Range2 const>::type
        >
    closest_pair_from_random_sample(Range1 const& r1,
                                    Range2 const& r2,
                                    Strategy const& strategy,
                                    Distance& dmin)
    {
        typedef typename boost::range_iterator<Range1 const>::type It1;
        typedef typename boost::range_iterator<Range2 const>::type It2;

        random_integer_generator<size_type> generator;

        size_type num_samples = boost::size(r1) + boost::size(r2);
        
        size_type i = generator(0, boost::size(r1) - 1);
        size_type j = generator(0, boost::size(r2) - 1);

        It1 it1_min = boost::begin(r1) + i;
        It2 it2_min = boost::begin(r2) + j;
        dmin = strategy.apply(*it1_min, *it2_min);

#if 0
#ifdef PRINT_DEBUG
        std::cout << "choosing points: "
                  << geometry::wkt(*(boost::begin(r1) + i))
                  << " "
                  << geometry::wkt(*(boost::begin(r2) + j))
                  << std::endl;
        std::cout << "distance: " << dmin << std::endl;
#endif
#endif
        for (size_type k = 1; k < num_samples; ++k) {
            i = generator(0, boost::size(r1) - 1);
            j = generator(0, boost::size(r2) - 1);
            Distance d = strategy.apply(*(boost::begin(r1) + i),
                                        *(boost::begin(r2) + j));
#if 0
#ifdef PRINT_DEBUG
            std::cout << "choosing points: "
                      << geometry::wkt(*(boost::begin(r1) + i))
                      << " "
                      << geometry::wkt(*(boost::begin(r2) + j))
                      << std::endl;
            std::cout << "distance: " << d << std::endl;
#endif
#endif
            if ( d < dmin )
            {
                it1_min = boost::begin(r1) + i;
                it2_min = boost::begin(r2) + j;
                dmin = d;
            }
        }
        return std::make_pair(it1_min, it2_min);
    }

    template <typename Point, int Dimension>
    struct coordinate_less
    {
        bool operator()(Point const& p, Point const& q) const
        {
            return geometry::get<Dimension>(p) < geometry::get<Dimension>(q);
        }
    };

    template <typename MP1, typename MP2>
    static inline typename point_type<MP1>::type
    compute_origin(MP1 const& mp1, MP2 const& mp2)
    {
        typedef typename point_type<MP1>::type P1;
        typedef typename point_type<MP2>::type P2;
        typedef typename coordinate_type<P1>::type C1;
        typedef typename coordinate_type<P2>::type C2;

        C1 xmin1 = geometry::get<0>(*std::min_element(boost::begin(mp1),
                                                      boost::end(mp1),
                                                      coordinate_less<P1,0>()
                                                      )
                                    );
        C2 xmin2 = geometry::get<0>(*std::min_element(boost::begin(mp2),
                                                      boost::end(mp2),
                                                      coordinate_less<P2,0>()
                                                      )
                                    );

        C1 ymin1 = geometry::get<1>(*std::min_element(boost::begin(mp1),
                                                      boost::end(mp1),
                                                      coordinate_less<P1,1>()
                                                      )
                                    );
        C2 ymin2 = geometry::get<1>(*std::min_element(boost::begin(mp2),
                                                      boost::end(mp2),
                                                      coordinate_less<P2,1>()
                                                      )
                                    );

        return P1(std::min(xmin1, xmin2), std::min(ymin1, ymin2));
    }


    template <typename Point, typename Origin, typename Distance>
    static inline Bin_id get_bin_id(Point const& p, Origin const& o,
                                    Distance const& d)
    {
        std::size_t x_id =
            std::floor( (geometry::get<0>(p) - geometry::get<0>(o)) / d );

        std::size_t y_id =
            std::floor( (geometry::get<1>(p) - geometry::get<1>(o)) / d );
        return std::make_pair(x_id, y_id);
    }

    template
    <
        typename MultiPoint, typename Bins,
        typename Origin, typename Distance
    >
    static inline void assign_to_bins(MultiPoint const& mp,
                                      Bins& bins,
                                      Origin const& o,
                                      Distance const& d)
    {
        typedef typename boost::range_iterator
            <
                MultiPoint const
            >::type Iterator;

        BOOST_AUTO_TPL(it, boost::begin(mp));
        for (; it != boost::end(mp); ++it)
        {
            Bin_id id = get_bin_id(*it, o, d);
            typename Bins::iterator bin_it = bins.find(id);
            if ( bin_it == bins.end() )
            {
                std::vector<Iterator> new_bin;
                new_bin.push_back(it);
                bins[id] = new_bin;
            } else {
                bin_it->second.push_back(it);
            }
        }
    }


    static void print_bin_id(Bin_id const& id)
    {
        std::cout << "(" << id.first << "," << id.second << "): ";
    }

    template <typename Bin>
    static void print_bin(Bin const& bin)
    {
        print_bin_id(bin.first);
        std::cout << geometry::wkt(bin.second) << std::endl;
    }

    template <typename Bins>
    static void print_bins(Bins const& bins, std::string const& str)
    {
        std::cout << std::endl;
        std::cout << str << std::endl;
        for (typename Bins::const_iterator it = bins.begin();
             it != bins.end(); ++it)
        {
            print_bin(*it);
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    template <typename Bins>
    static void print_bin_sizes(Bins const& bins, std::string const& str)
    {
        std::cout << std::endl;
        std::cout << str << std::endl;
        for (typename Bins::const_iterator it = bins.begin();
             it != bins.end(); ++it)
        {
            std::cout << " " << it->size();
        }
        std::cout << std::endl;
    }

    template
    <
        typename RedBin,
        typename BlueBins,
        typename Distance,
        typename Strategy
    >
    static inline void cp_to_blue_bin(RedBin const& red_bin,
                                      BlueBins const& blue_bins,
                                      Iterator1& it1_min,
                                      Iterator2& it2_min,
                                      Distance& dmin,
                                      int xoffset, int yoffset,
                                      Strategy const& strategy)
    {
        if ( (red_bin.first.first == 0 && xoffset < 0) ||
             (red_bin.first.second == 0 && yoffset < 0) )
        {
            return;
        }

        Bin_id blue_id = std::make_pair(red_bin.first.first + xoffset,
                                        red_bin.first.second + yoffset);
        
        typedef typename BlueBins::const_iterator BBIT;
        BBIT blue_bin = blue_bins.find(blue_id);

#if 0
#ifdef PRINT_DEBUG
            std::cout << "checking blue bin: ";
            print_bin_id(blue_id);
            std::cout << std::endl;
#endif
#endif
        if ( blue_bin != blue_bins.end() )
        {
            Distance d;
            return_type cp = brute_force_closest_pair
                <
                    Strategy
                >::apply(red_bin.second, blue_bin->second, strategy, d);
            if ( d < dmin )
            {
                it1_min = cp.first;
                it2_min = cp.second;
                dmin = d;
            }
        }
        else
        {
#if 0
#ifdef PRINT_DEBUG
            std::cout << "bin is empty" << std::endl;
#endif
#endif
        }
    }


    static inline return_type apply(MultiPoint1 const& multipoint1,
                                    MultiPoint2 const& multipoint2,
                                    Strategy const& strategy)
    {
        distance_type dmin;
        return_type cp = closest_pair_from_random_sample(multipoint1,
                                                         multipoint2,
                                                         strategy,
                                                         dmin);
        Iterator1 it1 = cp.first;
        Iterator2 it2 = cp.second;

        if ( dmin == 0 )
        {
            return std::make_pair(it1, it2);
        }

#if 1
        typedef typename std::map
            <
                Bin_id, std::vector<Iterator1>, Bin_id_less
            > Red_bins;
        typedef typename std::map
            <
                Bin_id, std::vector<Iterator2>, Bin_id_less
            > Blue_bins;
#else
        typedef typename boost::unordered_map //std::map
            <
                Bin_id, std::vector<Iterator1>
            > Red_bins;
        typedef typename boost::unordered_map //std::map
            <
                Bin_id, std::vector<Iterator2>
            > Blue_bins;
#endif

        Red_bins red_bins;
        Blue_bins blue_bins;

        typename point_type<MultiPoint1>::type origin
            = compute_origin(multipoint1, multipoint2);

#if 0
#ifdef PRINT_DEBUG
        std::cout << "My origin: " << geometry::wkt(origin) << std::endl;
#endif
#endif

        assign_to_bins(multipoint1, red_bins, origin, dmin);
        assign_to_bins(multipoint2, blue_bins, origin, dmin);

#if 0
#ifdef PRINT_DEBUG
        print_bins(red_bins, "Red Bins");
        print_bins(blue_bins, "Blue Bins");

        print_bin_sizes(red_bins, "Red bin sizes");
        print_bin_sizes(blue_bins, "Blue bin sizes");
        
        std::cout << std::endl << std::endl;
#endif
#endif
        for (typename Red_bins::const_iterator it = red_bins.begin();
             it != red_bins.end(); ++it)
        {
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin, -1, -1, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin, -1,  0, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin, -1,  1, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin,  0, -1, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin,  0,  0, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin,  0,  1, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin,  1, -1, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin,  1,  0, strategy);
            cp_to_blue_bin(*it, blue_bins, it1, it2, dmin,  1,  1, strategy);
        }
        return std::make_pair(it1, it2);
    }


    static inline return_type apply(MultiPoint1 const& multipoint1,
                                    MultiPoint2 const& multipoint2)
    {
        typedef typename detail::distance::default_strategy
            <
                typename point_type<MultiPoint1>::type,
                typename point_type<MultiPoint2>::type
            >::type Strategy;
        
        return apply(multipoint1, multipoint2, Strategy());
    }
};






}} // namespace detail::distance

}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_PAIR_HPP
