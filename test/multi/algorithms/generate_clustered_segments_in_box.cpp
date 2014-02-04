// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

#include <iostream>
#include <fstream>
#include <cassert>

#include "random_segment_generator.hpp"


int main(int argc, char** argv)
{
    assert( argc >= 4 );
    int n = atoi(argv[1]);
    int m = atoi(argv[2]);

    std::ofstream ofs( argv[3] );
    assert( ofs );

    ofs.precision(16);

    ofs << n << " " << m << " -1" << std::endl;

    random_segment_generator g;
    for (int i = 0; i < n; ++i)
    {
        random_segment_generator::segment s = g.apply(0, 0.05);
        ofs << s.first.first << " " << s.first.second
            << s.second.first << " " << s.second.second << std::endl;
    }
    for (int i = 0; i < m; ++i)
    {
        random_segment_generator::segment s = g.apply(0.95, 1);
        ofs << s.first.first << " " << s.first.second
            << s.second.first << " " << s.second.second << std::endl;
    }

    ofs.close();

    return 0;
}
