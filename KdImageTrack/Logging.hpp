//
//  Logging.hpp
//  VisionDev-demo
//
//  Created on 19/01/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//
// This is to provide alternatives to printf which can be used to print different types of message, and turned on and off independently



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
