/**
 *    This file is part of KdImageTrack.
 *
 *   KdImageTrack is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   KdImageTrack is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with KdImageTrack. If not, see <https://www.gnu.org/licenses/>.
 *
 *    Copyright (c) 2015 Kudan. All rights reserved.
 **/

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
