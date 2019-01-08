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
