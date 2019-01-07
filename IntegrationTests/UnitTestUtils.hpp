//
//  UnitTestUtils.hpp
//  KdImageTrack
//
//  Created on 29/09/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#ifndef UnitTestUtils_hpp
#define UnitTestUtils_hpp

#include <stdio.h>

#include <string>


const bool doSlowTests = false;


/** This just reads the first line from a file and returns it
 */
std::string loadKey(std::string file);

#endif /* UnitTestUtils_hpp */
