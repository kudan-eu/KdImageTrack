//
//  Logging.cpp
//  VisionDev-demo
//
//  Created on 19/01/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//


#include "Logging.hpp"
#include <stdarg.h>

#ifdef __ANDROID__
#include <android/log.h>
#endif


Namespace(KdImageTrack)


void printlog(int logCode, const char *fmt, ... )
{
    
    
#ifdef SHOW_LOGS
   
    /* Declare a va_list type variable */
    va_list myargs;
    
    /* Initialise the va_list variable with the ... after fmt */
    va_start(myargs, fmt);

    
#ifdef __APPLE__

    // the log code is going to be one of the hash defined things, which may be set to 0 or 1
    if (logCode!=0) {

        
        if (logCode > 0) {
            if (logCode & LOG_DETECT){
                printf("detector: ");
            }
            if (logCode & LOG_TRACK) {
                printf("tracker: ");
            }
            
            if (logCode & LOG_PATCH) {
                printf("patches: ");
            }
            if (logCode & LOG_FLOW) {
                printf("optical flow: ");
            }
            if (logCode & LOG_DRAW) {
                printf("drawing: ");
            }
            if (logCode & LOG_HOMOGRAPHY) {
                printf("homography: ");
            }
            if (logCode & LOG_EXT) {
                printf("extensions: ");
            }
            if (logCode & LOG_RECOVER) {
                printf("recovery: ");
            }
        }
        
        vprintf(fmt, myargs);
        
    }
    
#else
    
    std::string logTag = "";
    if (logCode > 0) {
        if (logCode & LOG_DETECT) {
            logTag = "detector: ";
        }
        if (logCode & LOG_TRACK) {
            logTag = "tracker: ";
        }
        
        if (logCode & LOG_PATCH) {
            logTag = "patches: ";
        }
        if (logCode & LOG_FLOW) {
            logTag = "optical flow: ";
        }
        if (logCode & LOG_DRAW) {
            logTag = "drawing: ";
        }
        if (logCode & LOG_HOMOGRAPHY) {
            logTag = "homography: ";
        }
        if (logCode & LOG_EXT) {
            logTag = "extensions: ";
        }
        if (logCode & LOG_RECOVER) {
            logTag = "recovery: ";
        }
    }

    __android_log_vprint(ANDROID_LOG_DEBUG, logTag.c_str(), fmt, myargs);
    
    
#endif
    
    /* Clean up the va_list */
    va_end(myargs);
    
#endif

    
}

void printerr(const char *fmt, ... )
{


#ifdef SHOW_ERRORS
    
    /* Declare a va_list type variable */
    va_list myargs;
    
    /* Initialise the va_list variable with the ... after fmt */
    va_start(myargs, fmt);
    
#ifdef __APPLE__

    
    printf("*ERROR*: ");
    
    
    vprintf(fmt, myargs);
    
    
#else
    __android_log_vprint(ANDROID_LOG_ERROR, "*ERROR*: ", fmt, myargs);
    
    
#endif
    
    /* Clean up the va_list */
    va_end(myargs);
    
#endif
    
}


void printdebug(const char *fmt, ... )
{
    
#ifdef SHOW_DEBUG
    
    /* Declare a va_list type variable */
    va_list myargs;
    
    /* Initialise the va_list variable with the ... after fmt */
    va_start(myargs, fmt);

    
#ifdef __APPLE__
    
    printf("DEBUG: ");
    
    vprintf(fmt, myargs);
    
    
#else
    __android_log_vprint(ANDROID_LOG_DEBUG, "DEBUG: ", fmt, myargs);
    
    
#endif
    
    
    /* Clean up the va_list */
    va_end(myargs);
    
#endif
    

}


void printtime(const char *fmt, ... )
{
    
#ifdef SHOW_TIMING
    
    /* Declare a va_list type variable */
    va_list myargs;
    
    /* Initialise the va_list variable with the ... after fmt */
    va_start(myargs, fmt);
    
    
#ifdef __APPLE__

    
    printf("!! ");
    
    vprintf(fmt, myargs);
    
    
#else
    __android_log_vprint(ANDROID_LOG_DEBUG, "!! ", fmt, myargs);
    
#endif
    
    /* Clean up the va_list */
    va_end(myargs);
    
    
#endif
    
}


NamespaceEnd
