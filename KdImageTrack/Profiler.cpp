//
//  Profiler.cpp
//  KdImageTrack
//
//  Created on 17/11/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#include "Profiler.hpp"

#ifdef __APPLE__
#include "TargetConditionals.h"
#endif

extern "C"
{
#if TARGET_OS_IOS
    double CACurrentMediaTime();
#endif
}


Namespace(KdImageTrack)

void Profiler::start()
{
#if TARGET_OS_IOS
    startTime = CACurrentMediaTime();
#else
    
#endif
}

void Profiler::end()
{
#if TARGET_OS_IOS
    endTime = CACurrentMediaTime();
#else
    
#endif
}

double Profiler::getTimeTaken()
{
#if TARGET_OS_IOS
    return endTime - startTime;
#else    
    return 0;
#endif
}

NamespaceEnd
