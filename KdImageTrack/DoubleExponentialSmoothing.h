//
//  DoubleExponentialSmoothing.h
//  PlanarTracker
//
//  Created on 27/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#ifndef __PlanarTracker__DoubleExponentialSmoothing__
#define __PlanarTracker__DoubleExponentialSmoothing__

#include <stdio.h>
#include <assert.h>
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/quaternion.hpp"
#include "Macros.h"

Namespace(KdImageTrack)

class DoubleExponentialSmoothingOrientation
{
private:
    float alpha;
    float beta;
    
    glm::quat s;
    glm::quat b;
    
    glm::quat prevS;
    glm::quat prevB;
    
    glm::quat x;
    glm::quat prevX;
    
    int numberOfObservations;
public:
    DoubleExponentialSmoothingOrientation()
    {
        alpha = 0.8;
        beta = 0.9;
        numberOfObservations = 0;
    }
    
    DoubleExponentialSmoothingOrientation(float a, float b)
    {
        alpha = a;
        beta = b;
        numberOfObservations = 0;
    }
    
    void setObserved(glm::quat observed);
    glm::quat getSmoothed();
    
    void reset()
    {
        numberOfObservations = 0;
    }

};

class DoubleExponentialSmoothing
{
private:
    float alpha;
    float beta;
    
    float s;
    float b;
    
    float x;
    float prevX;
    
    int numberOfObservations;
public:
    float prevS;
    float prevB;

    
    DoubleExponentialSmoothing()
    {
        alpha = 0.8;
        beta = 0.9;
        numberOfObservations = 0;
    }
    
    DoubleExponentialSmoothing(float a, float b)
    {
        alpha = a;
        beta = b;
        numberOfObservations = 0;
    }
    
    void setObserved(float observed);
    float getSmoothed();
    
    void reset()
    {
        numberOfObservations = 0;
    }
};

class DoubleExponentialSmoothingTracker
{
    DoubleExponentialSmoothing x;
    DoubleExponentialSmoothing y;
    DoubleExponentialSmoothing z;
    
    DoubleExponentialSmoothingOrientation o;
public:
    DoubleExponentialSmoothingTracker()
    {
    }
    
    void setState(glm::vec3 position, glm::quat orientation)
    {
        x.setObserved(position.x);
        y.setObserved(position.y);
        z.setObserved(position.z);

        o.setObserved(orientation);
    }
    
    glm::vec3 getSmoothedPosition()
    {
        return glm::vec3(x.getSmoothed(), y.getSmoothed(), z.getSmoothed());
    }
    
    glm::quat getSmoothedOrientation()
    {
        return o.getSmoothed();
    }
    
    void reset()
    {
        x.reset();
        y.reset();
        z.reset();
        
        o.reset();
    }

};

NamespaceEnd

#endif /* defined(__PlanarTracker__DoubleExponentialSmoothing__) */
