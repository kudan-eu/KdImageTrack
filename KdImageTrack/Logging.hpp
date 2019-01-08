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



#ifndef Logging_hpp
#define Logging_hpp

#include <stdio.h>
#include <iostream>
#include "Macros.h"

Namespace(KdImageTrack)


#define SHOW_LOGS
#define SHOW_ERRORS
//#define SHOW_DEBUG
//#define SHOW_TIMING

#ifdef SHOW_LOGS
#define logfunc printf(">> Function %s at line %i << \n", __func__, __LINE__);
#else
#define logfunc
#endif



#define LOG_DETECT 1*0
#define LOG_TRACK 2*0
#define LOG_PATCH 8*0
#define LOG_DRAW 16*0
#define LOG_FLOW 32*0
#define LOG_HOMOGRAPHY 64*0
#define LOG_EXT 128*0
#define LOG_RECOVER 256*0

void printlog(int logCode, const char *fmt, ... );

void printerr(const char *fmt, ... );
void printdebug(const char *fmt, ... );
void printtime(const char *fmt, ... );

NamespaceEnd

#endif /* Logging_hpp */
