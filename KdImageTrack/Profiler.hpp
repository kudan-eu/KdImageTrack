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
