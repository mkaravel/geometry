// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cassert>
#include <utility>

#include "random_segment_generator.hpp"


int main(int argc, char** argv)
{
    assert( argc >= 4 );
    int n = atoi(argv[1]);
    int m = atoi(argv[2]);

    std::ofstream ofs( argv[3] );
    assert( ofs );

    double max_length = -1;
    if ( argc > 4 )
    {
        max_length = atof(argv[4]);
        std::cout << "maximum length of generated segments: "
                  << max_length << std::endl;
    }

    ofs.precision(16);

    ofs << n << " " << m << " " << max_length << std::endl;

    random_segment_generator g(max_length);
    for (int i = 0; i < n + m; ++i)
    {
        random_segment_generator::segment s = *g;
        ofs << s.first.first << " " << s.first.second << " "
            << s.second.first << " " << s.second.second << std::endl;
    }

    ofs.close();

    return 0;
}
