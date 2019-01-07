//
//  DoubleExponentialSmoothing.cpp
//  PlanarTracker
//
//  Created on 27/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#include "DoubleExponentialSmoothing.h"

Namespace(KdImageTrack)

void DoubleExponentialSmoothingOrientation::setObserved(glm::quat observed)
{
    if (numberOfObservations > 0) {
        prevX = x;
    }
    
    x = observed;
    numberOfObservations++;
}

static glm::quat diff(glm::quat q1, glm::quat q2)
{
    return q2 * glm::inverse(q1);
}

glm::quat DoubleExponentialSmoothingOrientation::getSmoothed()
{
    if (numberOfObservations == 1) {
        return x;
    }
    if (numberOfObservations == 2) {
        s = x;
        b = diff(x, prevX);
        return s;
    }
    
    // numberOfObservations > 1
    glm::quat identity;
    
    prevS = s;
    s = glm::slerp(identity, x, alpha) * glm::slerp(identity, b * s, 1 - alpha);
    s = glm::normalize(s);
    
    prevB = b;
    b = glm::slerp(identity, diff(s, prevS), beta) * glm::slerp(identity, prevB, 1 - beta);
    b = glm::normalize(b);
    
    return s;
}



void DoubleExponentialSmoothing::setObserved(float observed)
{
    if (numberOfObservations > 0) {
        prevX = x;
    }
    
    x = observed;
    numberOfObservations++;
}

float DoubleExponentialSmoothing::getSmoothed()
{
    if (numberOfObservations == 1) {
        return x;
    }
    
    if (numberOfObservations == 2) {
        s = x;
        b = x - prevX;
        return s;
    }
    
    // numberOfObservations > 1
    prevS = s;
    s = alpha * x + (1 - alpha) * (s + b);
    
    prevB = b;
    b = beta * (s - prevS) + (1 - beta) * prevB;
    
    return s;
}

NamespaceEnd
