// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#ifndef BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_HPP
#define BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_HPP

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/multi/algorithms/distance.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/random_integer_generator.hpp>
#include <boost/geometry/multi/algorithms/detail/distance/extremal_coordinate.hpp>

#include <cmath>
#include <functional>


namespace boost { namespace geometry
{

namespace detail { namespace distance
{

// assignment to bins
struct assign_to_bins_base
{
    template <typename Pair>
    struct pair_less
    { 
        inline bool operator()(Pair const& pair1, Pair const& pair2) const
        {
            if ( pair1.first != pair2.first )
            {
                return pair1.first < pair2.first;
            }
            return pair1.second < pair2.second;
        }
    };

    typedef std::size_t size_type;
    typedef std::pair<size_type,size_type> Bin_id;
    typedef pair_less<Bin_id> Bin_id_less;

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
};



template
<
    typename MultiPoint, typename Bins,
    typename Origin, typename Distance
>
struct assign_to_bins_multi_point
    : public assign_to_bins_base
{
    typedef assign_to_bins_base Base;

    static inline void apply(MultiPoint const& mp, Bins& bins,
                             Origin const& o, Distance const& d)
    {
        BOOST_AUTO_TPL(it, boost::begin(mp));
        for (; it != boost::end(mp); ++it)
        {
            Bin_id id = Base::get_bin_id(*it, o, d);
            typename Bins::iterator bin_it = bins.find(id);
            if ( bin_it == bins.end() )
            {
                MultiPoint new_bin;
                new_bin.push_back(*it);
                bins[id] = new_bin;
            } else {
                bin_it->second.push_back(*it);
            }
        }
    }
};



template
<
    typename SegmentRange, typename Bins,
    typename Origin, typename Distance
>
struct assign_to_bins_segment_range
    : public assign_to_bins_base
{
    typedef assign_to_bins_base Base;

    template <typename Point, typename Segment>
    static inline void assign_segment_to_bin(Point const& point,
                                             Segment const& segment,
                                             Bins& bins,
                                             Origin const& o,
                                             Distance const& d)
    {
        Bin_id id = Base::get_bin_id(point, o, d);
        typename Bins::iterator bin_it = bins.find(id);
        if ( bin_it == bins.end() )
        {
            std::vector<Segment> new_bin;
            new_bin.push_back(segment);
            bins[id] = new_bin;
        } else {
            bin_it->second.push_back(segment);
        }
    }


    template <typename Segment>
    static inline void assign_segment_to_bins(Segment const& segment,
                                              Bins& bins,
                                              Origin const& o,
                                              Distance const& d)
    {
        typedef typename point_type<Segment>::type Point;
        typedef typename coordinate_type<Segment>::type coordinate_type;

        Distance length = geometry::length(segment);
        //        std::size_t num_sub_segments = std::floor(2 * length / d) + 1;
        std::size_t num_sub_segments = std::floor((5 * length) / (4 * d)) + 1;
        
        coordinate_type x_src = geometry::get<0,0>(segment);
        coordinate_type y_src = geometry::get<0,1>(segment);
        coordinate_type x_trg = geometry::get<1,0>(segment);
        coordinate_type y_trg = geometry::get<1,1>(segment);
        
        Point ends[2];
        detail::assign_point_from_index<0>(segment, ends[0]);
        detail::assign_point_from_index<1>(segment, ends[1]);
        assign_segment_to_bin(ends[0], segment, bins, o, d);
        assign_segment_to_bin(ends[1], segment, bins, o, d);

        for (std::size_t i = 1; i < num_sub_segments; ++i)
        {
            Point p(x_src + i * (x_trg - x_src) / num_sub_segments,
                    y_src + i * (y_trg - y_src) / num_sub_segments);
            assign_segment_to_bin(p, segment, bins, o, d);
        }
    }


    static inline void apply(SegmentRange const& segments, Bins& bins,
                             Origin const& o, Distance const& d)
    {
        BOOST_AUTO_TPL(it, boost::begin(segments));
        for (; it != boost::end(segments); ++it)
        {
            assign_segment_to_bins(*it, bins, o, d);
        }
    }
};



template
<
    typename Geometry, typename Range, typename Bins, typename Origin,
    typename Distance, typename GeometryTag
>
struct assign_to_bins_dispatch
    : not_implemented<Geometry>
{};


template
<
    typename MultiPoint, typename Range, typename Bins,
    typename Origin, typename Distance
>
struct assign_to_bins_dispatch
    <
        MultiPoint, Range, Bins, Origin, Distance, multi_point_tag
    > : assign_to_bins_multi_point<MultiPoint, Bins, Origin, Distance>
{};


template
<
    typename LineString, typename SegmentRange,
    typename Bins, typename Origin, typename Distance
>
struct assign_to_bins_dispatch
    <
        LineString, SegmentRange, Bins, Origin, Distance, linestring_tag
    > : assign_to_bins_segment_range<SegmentRange, Bins, Origin, Distance>
{};


template
<
    typename MultiLineString, typename SegmentRange, typename Bins,
    typename Origin, typename Distance
>
struct assign_to_bins_dispatch
    <
        MultiLineString, SegmentRange, Bins, Origin, Distance,
        multi_linestring_tag
    > : assign_to_bins_segment_range<SegmentRange, Bins, Origin, Distance>
{};


// assign to bins function
template
<
    typename Geometry, typename Range, typename Bins,
    typename Origin, typename Distance
>
void assign_to_bins(Range const& range, Bins& bins,
                    Origin const& origin, Distance const& d)
{
    assign_to_bins_dispatch
        <
            Geometry, Range, Bins, Origin, Distance,
            typename tag<Geometry>::type
        >::apply(range, bins, origin, d);
};




// brute force distance computation
template <typename Range1, typename Range2, typename Strategy>
struct brute_force_distance
{
    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Range1>::type,
            typename point_type<Range2>::type
        >::type return_type;

    static inline return_type
    apply(Range1 const& r1, Range2 const& r2, Strategy const& strategy)
    {
        BOOST_AUTO_TPL(it1, boost::begin(r1));

        return_type dmin =
            geometry::distance(*it1, *boost::begin(r2), strategy);

        for (; it1 != boost::end(r1); ++it1)
        {
            BOOST_AUTO_TPL(it2, boost::begin(r2));
            for (; it2 != boost::end(r2); ++it2)
            {
                return_type d = geometry::distance(*it1, *it2, strategy);
                if ( d < dmin )
                {
                    dmin = d;
                }
            }
        }

        return dmin;
    }
};



template <typename Geometry1, typename Geometry2, typename Strategy>
struct closest_distance_base
    : assign_to_bins_base
{
    typedef assign_to_bins_base Base;

    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type return_type;


    template <typename Range1, typename Range2>
    static inline return_type
    brute_force_distance(Range1 const& r1, Range2 const& r2,
                         Strategy const& strategy)
    {
        BOOST_AUTO_TPL(it1, boost::begin(r1));

        return_type dmin =
            geometry::distance(*it1, *boost::begin(r2), strategy);

        for (; it1 != boost::end(r1); ++it1)
        {
            BOOST_AUTO_TPL(it2, boost::begin(r2));
            for (; it2 != boost::end(r2); ++it2)
            {
                return_type d = geometry::distance(*it1, *it2, strategy);
                if ( d < dmin )
                {
                    dmin = d;
                }
            }
        }

        return dmin;
    }


    template <typename Range1, typename Range2>
    static inline return_type
    distance_from_random_sample(Range1 const& r1,
                                Range2 const& r2,
                                Strategy const& strategy)
    {
        random_integer_generator<size_type> generator;

        size_type num_samples = boost::size(r1) + boost::size(r2);
        
        size_type i = generator(0, boost::size(r1) - 1);
        size_type j = generator(0, boost::size(r2) - 1);
        return_type dmin = geometry::distance(*(boost::begin(r1) + i),
                                              *(boost::begin(r2) + j),
                                              strategy);

#ifdef PRINT_DEBUG
        std::cout << "choosing points: "
                  << geometry::wkt(*(boost::begin(r1) + i))
                  << " "
                  << geometry::wkt(*(boost::begin(r2) + j))
                  << std::endl;
        std::cout << "distance: " << dmin << std::endl;
#endif
        return_type d;
        for (size_type k = 1; k < num_samples; ++k) {
            i = generator(0, boost::size(r1) - 1);
            j = generator(0, boost::size(r2) - 1);
            d = geometry::distance(*(boost::begin(r1) + i),
                                   *(boost::begin(r2) + j),
                                   strategy);

#ifdef PRINT_DEBUG
            std::cout << "choosing points: "
                      << geometry::wkt(*(boost::begin(r1) + i))
                      << " "
                      << geometry::wkt(*(boost::begin(r2) + j))
                      << std::endl;
            std::cout << "distance: " << d << std::endl;
#endif
            if ( d < dmin )
            {
                dmin = d;
            }
            if ( dmin == 0 )
            {
                return 0;
            }
        }
        return dmin;
    }




    template <typename Point, int Dimension>
    struct coordinate_less
    {
        bool operator()(Point const& p, Point const& q) const
        {
            return geometry::get<Dimension>(p) < geometry::get<Dimension>(q);
        }
    };

    template <typename Range1, typename Range2>
    static inline typename point_type<Range1>::type
    compute_origin(Range1 const& r1, Range2 const& r2)
    {
        typedef typename point_type<Range1>::type P1;
        typedef typename point_type<Range2>::type P2;
        typedef typename coordinate_type<Range1>::type C1;
        typedef typename coordinate_type<Range2>::type C2;

        C1 xmin1 = extremal_coordinate<0>(r1);
        C2 xmin2 = extremal_coordinate<0>(r2);
        C1 ymin1 = extremal_coordinate<1>(r1);
        C2 ymin2 = extremal_coordinate<1>(r2);

        return P1(std::min(xmin1, xmin2), std::min(ymin1, ymin2));
    }


    static void print_bin_id(Bin_id const& id)
    {
        std::cout << "(" << id.first << "," << id.second << "): ";
    }

    template <typename Bin>
    static void print_bin(Bin const& bin)
    {
        print_bin_id(bin.first);
        BOOST_AUTO_TPL(it, boost::begin(bin.second));
        for (; it != boost::end(bin.second); ++it)
        {
            std::cout << geometry::wkt(*it) << ' ';
        }
        std::cout << std::endl;
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
        }
    }

    template <typename Bins>
    static void print_bin_sizes(Bins const& bins, std::string const& str)
    {
        std::cout << std::endl;
        std::cout << str << std::endl;
        for (typename Bins::const_iterator it = bins.begin();
             it != bins.end(); ++it)
        {
            std::cout << " ";
            print_bin_id(it->first);
            std::cout << " " << it->second.size();
        }
        std::cout << std::endl;
    }


    template
    <
        typename RedBin,
        typename BlueBins,
        typename Distance
    >
    static inline void eval_distance_to_blue_bin(RedBin const& red_bin,
                                                 BlueBins const& blue_bins,
                                                 Distance& dmin,
                                                 int xoffset, int yoffset,
                                                 Strategy const& strategy)
    {
        //        std::cout << "red bin size: " << red_bin.second.size() << std::endl;
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
        if ( blue_bin != blue_bins.end() )
        {
            std::cout << "blue bin size: " << blue_bin->second.size() << std::endl;
        }
        else
        {
            std::cout << "blue bin not found" << std::endl;
        }
#endif
#ifdef PRINT_DEBUG
            std::cout << "checking blue bin: ";
            print_bin_id(blue_id);
            std::cout << std::endl;
#endif
        if ( blue_bin != blue_bins.end() )
        {
            Distance d = brute_force_distance(red_bin.second,
                                              blue_bin->second,
                                              strategy);
            if ( d < dmin )
            {
                dmin = d;
            }
        }
        else
        {
#ifdef PRINT_DEBUG
            std::cout << "bin is empty" << std::endl;
#endif
        }
    }
};



// closest distance algorithm for multi-points
template
<
    typename MultiPoint1,
    typename MultiPoint2,
    typename Strategy
>
struct closest_distance_multi_point
    : closest_distance_base<MultiPoint1, MultiPoint2, Strategy>
{
    typedef closest_distance_base<MultiPoint1, MultiPoint2, Strategy> Base;

    typedef typename Base::return_type return_type;
    typedef typename Base::Bin_id Bin_id;
    typedef typename Base::Bin_id_less Bin_id_less;

    static inline return_type apply(MultiPoint1 const& multipoint1,
                                    MultiPoint2 const& multipoint2,
                                    Strategy const& strategy)
    {
        return_type dmin = Base::distance_from_random_sample(multipoint1,
                                                             multipoint2,
                                                             strategy);

        std::cout << "closest distance estimate: " << dmin << std::endl;

        typedef typename std::map<Bin_id, MultiPoint1, Bin_id_less> Red_bins;
        typedef typename std::map<Bin_id, MultiPoint2, Bin_id_less> Blue_bins;
        Red_bins red_bins;
        Blue_bins blue_bins;

        typename point_type<MultiPoint1>::type origin
            = Base::compute_origin(multipoint1, multipoint2);

        std::cout << "My origin: " << geometry::wkt(origin) << std::endl;

#ifdef PRINT_DEBUG
        std::cout << "My origin: " << geometry::wkt(origin) << std::endl;
#endif

        assign_to_bins<MultiPoint1>(multipoint1, red_bins, origin, dmin);
        assign_to_bins<MultiPoint2>(multipoint2, blue_bins, origin, dmin);

#if 0
        std::cout << std::endl << std::endl;
        Base::print_bin_sizes(red_bins, "Red bin sizes");
        Base::print_bin_sizes(blue_bins, "Blue bin sizes");
        std::cout << std::endl << std::endl;
#endif

#ifdef PRINT_DEBUG
        print_bins(red_bins, "Red Bins");
        print_bins(blue_bins, "Blue Bins");

        print_bin_sizes(red_bins, "Red bin sizes");
        print_bin_sizes(blue_bins, "Blue bin sizes");

        std::cout << std::endl << std::endl;
#endif
        for (typename Red_bins::const_iterator it = red_bins.begin();
             it != red_bins.end(); ++it)
        {
            for (int i = -1; i < 2; ++i)
            {
                for (int j = -1; j < 2; ++j)
                {
                    Base::eval_distance_to_blue_bin(*it, blue_bins, dmin,
                                                    i, j, strategy);
                }
            }
        }
        return dmin;
    }
};



// closest distance algorithm for linear geometries
template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy
>
struct closest_distance_linear
    : closest_distance_base<Geometry1, Geometry2, Strategy>
{
    typedef closest_distance_base<Geometry1, Geometry2, Strategy> Base;

    template <typename LineString, typename OutputIterator>
    static inline
    OutputIterator split_to_segments(LineString const& ls,
                                     OutputIterator oit)
    {
        typedef geometry::model::segment
            <
                typename point_type<LineString>::type
            > Segment;

        BOOST_AUTO_TPL(it1, boost::begin(ls));
        BOOST_AUTO_TPL(it2, boost::begin(ls));

        for (++it2; it2 != boost::end(ls); ++it1, ++it2)
        {
            *oit++ = Segment(*it1, *it2);
        }

        return oit;
    }

    template <typename MultiLineString, typename OutputIterator>
    static inline
    OutputIterator split_to_segments(MultiLineString const& mls,
                                     OutputIterator oit, int)
    {
        BOOST_AUTO_TPL(it, boost::begin(mls));
        for (; it != boost::end(mls); ++it)
        {
            oit = split_to_segments(*it, oit);
        }

        return oit;
    }

    typedef typename Base::return_type return_type;
    typedef typename Base::Bin_id Bin_id;
    typedef typename Base::Bin_id_less Bin_id_less;


    static inline
    typename Base::return_type apply(Geometry1 const& geometry1,
                                     Geometry2 const& geometry2,
                                     Strategy const& strategy)
    {
        typedef typename point_type<Geometry1>::type Point1;
        typedef typename point_type<Geometry2>::type Point2;
        typedef geometry::model::segment<Point1> Segment1;
        typedef geometry::model::segment<Point2> Segment2;

        typedef std::vector<Segment1> SegmentRange1;
        typedef std::vector<Segment2> SegmentRange2;
        SegmentRange1 segments1;
        SegmentRange2 segments2;

        split_to_segments(geometry1, std::back_inserter(segments1), 0);
        split_to_segments(geometry2, std::back_inserter(segments2), 0);

#ifdef PRINT_DEBUG
        std::cout << "segments #1: " << boost::size(segments1) << std::endl;
        std::cout << "segments #2: " << boost::size(segments2) << std::endl;
#endif

        return_type dmin = Base::distance_from_random_sample(segments1,
                                                             segments2,
                                                             strategy);

        std::cout << "closest distance estimate: " << dmin << std::endl;

        if ( dmin == 0 )
        {
            return 0;
        }

        typedef typename std::map<Bin_id, SegmentRange1, Bin_id_less> Red_bins;
        typedef typename std::map<Bin_id, SegmentRange2, Bin_id_less> Blue_bins;
        Red_bins red_bins;
        Blue_bins blue_bins;

        typename point_type<Geometry1>::type origin
            = Base::compute_origin(geometry1, geometry2);

        std::cout << "My origin: " << geometry::wkt(origin) << std::endl;

        assign_to_bins<Geometry1>(segments1, red_bins, origin, dmin);
        assign_to_bins<Geometry2>(segments2, blue_bins, origin, dmin);

        for (typename Red_bins::const_iterator it = red_bins.begin();
             it != red_bins.end(); ++it)
        {
            for (int i = -1; i < 2; ++i)
            {
                for (int j = -1; j < 2; ++j)
                {
                    Base::eval_distance_to_blue_bin(*it, blue_bins, dmin,
                                                    i, j, strategy);
                    if ( dmin == 0 )
                    {
                        return 0;
                    }
                }
            }
        }
        return dmin;
    }
};








template
<
    typename Geometry1,
    typename Geometry2,
    typename Strategy,
    typename GeometryTag1,
    typename GeometryTag2
>
struct closest_distance_dispatch
    : not_implemented<Geometry1, Geometry2, Strategy>
{};

// dispatch
template <typename MultiPoint1, typename MultiPoint2, typename Strategy>
struct closest_distance_dispatch
    <
        MultiPoint1, MultiPoint2, Strategy,
        multi_point_tag, multi_point_tag
    > : closest_distance_multi_point<MultiPoint1, MultiPoint2, Strategy>
{};


template <typename LineString1, typename LineString2, typename Strategy>
struct closest_distance_dispatch
    <
        LineString1, LineString2, Strategy,
        linestring_tag, linestring_tag
    > : closest_distance_linear<LineString1, LineString2, Strategy>
{};


template
<
    typename MultiLineString1, typename MultiLineString2, typename Strategy
>
struct closest_distance_dispatch
    <
        MultiLineString1, MultiLineString2, Strategy,
        multi_linestring_tag, multi_linestring_tag
    > : closest_distance_linear<MultiLineString1, MultiLineString2, Strategy>
{};




// generic class
template
<
    typename Geometry1,
    typename Geometry2,
    typename PointToPointStrategy = typename detail::distance::default_strategy
    <
        typename point_type<Geometry1>::type,
        typename point_type<Geometry2>::type
    >::type
>
struct closest_distance
{
    typedef PointToPointStrategy Strategy;

    typedef typename strategy::distance::services::return_type
        <
            Strategy,
            typename point_type<Geometry1>::type,
            typename point_type<Geometry2>::type
        >::type return_type;


    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2,
                                    Strategy const& strategy)
    {
        return closest_distance_dispatch
            <
                Geometry1, Geometry2, Strategy,
                typename tag<Geometry1>::type, typename tag<Geometry2>::type
            >::apply(geometry1, geometry2, strategy);
    }

    static inline return_type apply(Geometry1 const& geometry1,
                                    Geometry2 const& geometry2)
    {
        

        return closest_distance_dispatch
            <
                Geometry1, Geometry2, Strategy,
                typename tag<Geometry1>::type, typename tag<Geometry2>::type
            >::apply(geometry1, geometry2, Strategy());
    }
};




}} // namespace detail::distance

}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_MULTI_ALGORITHMS_DETAIL_DISTANCE_CLOSEST_DISTANCE_HPP
