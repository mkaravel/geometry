// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

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
    for (int i = 0; i < n + m; ++i)
    {
        random_point_generator::point p = *g;
        ofs << p.first << " " << p.second << std::endl;
    }

    ofs.close();

    return 0;
}
