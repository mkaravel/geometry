// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#include <iostream>
#include <fstream>
#include <cassert>

#include "random_point_generator.hpp"


int main(int argc, char** argv)
{
    assert( argc >= 4 );
    int n = atoi(argv[1]);
    int m = atoi(argv[2]);

    std::ofstream ofs( argv[3] );
    assert( ofs );

    ofs.precision(16);

    ofs << n << " " << m << std::endl;

    random_point_generator g;
    for (int i = 0; i < n / 2; ++i)
    {
        random_point_generator::point p = g.apply(0, 0.01);
        ofs << p.first << " " << p.second << std::endl;
    }
    for (int i = n / 2; i < n; ++i)
    {
        random_point_generator::point p = g.apply(0.99, 1, 0, 0.01);
        ofs << p.first << " " << p.second << std::endl;
    }
    for (int i = 0; i < m / 2; ++i)
    {
        random_point_generator::point p = g.apply(0.99, 1);
        ofs << p.first << " " << p.second << std::endl;
    }
    for (int i = m / 2; i < m; ++i)
    {
        random_point_generator::point p = g.apply(0, 0.01, 0.99, 1);
        ofs << p.first << " " << p.second << std::endl;
    }

    ofs.close();

    return 0;
}
