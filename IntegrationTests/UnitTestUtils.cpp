//
//  UnitTestUtils.cpp
//  KdImageTrack
//
//  Created on 29/09/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#include "UnitTestUtils.hpp"

#include <fstream>


std::string loadKey(std::string file)
{
    std::ifstream fileStream;
    fileStream.open(file);
    if (fileStream.is_open()) {
        std::string line;
        getline(fileStream, line);
        return line;
    }
    else {
        printf("COULD NOT OPEN FILE! \n");
        return "";
    }
    
}
