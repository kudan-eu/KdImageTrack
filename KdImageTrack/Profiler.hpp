//
//  Profiler.hpp
//  KdImageTrack
//
//  Created on 17/11/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#ifndef Profiler_hpp
#define Profiler_hpp

#include <stdio.h>
#include <time.h>
#include "Macros.h"

Namespace(KdImageTrack)



class Profiler
{
private:
    double startTime;
    double endTime;

    time_t m_timeStart;
    time_t m_timeFinish;
 
public:
    Profiler()
    {
        startTime = 0;
        endTime = 0;
    }
    
    void start();
    void end();
    double getTimeTaken();
};

NamespaceEnd

#endif /* Profiler_hpp */
