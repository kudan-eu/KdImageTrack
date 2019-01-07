//
//  Test_KudanMaths.cpp
//  KdImageTrack
//
//  Created on 03/11/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//


// This is for testing the (limited_ range of maths functions / types in KdImageTrack, e.g. KudanMatrix4

#include <stdio.h>



#include <stdio.h>
#include "catch.hpp"
#include "KdImageTrack.h"


TEST_CASE( "Kudan Maths tests", "[Maths]" ) {
    
    GIVEN ("A KudanVector2") {
        
        WHEN ("Not initialised") {
            KudanVector2 vec2;
            THEN ("All values should be zero") {
                REQUIRE(vec2.x == 0.f);
                REQUIRE(vec2.y == 0.f);
            }
        }
        WHEN ("Constructing with arguments") {
            KudanVector2 vec2(1,4);
            THEN ("Its values should be set") {
                REQUIRE(vec2.x == 1.f);
                REQUIRE(vec2.y == 4.f);
            }
       
            THEN ("The zero function should reset it") {
                vec2.zero();
                REQUIRE(vec2.x == 0.f);
                REQUIRE(vec2.y == 0.f);
            }
        }
    }
    GIVEN ("A KudanVector3") {
        
        WHEN ("Not initialised") {
            KudanVector3 vec3;
            THEN ("All values should be zero") {
                REQUIRE(vec3.x == 0.f);
                REQUIRE(vec3.y == 0.f);
                REQUIRE(vec3.z == 0.f);
            }
        }
        WHEN ("Constructing with arguments") {
            KudanVector3 vec3(9, 5.5, 2);
            THEN ("Its values should be set") {
                REQUIRE(vec3.x == 9.f);
                REQUIRE(vec3.y == 5.5f);
                REQUIRE(vec3.z == 2.f);
            }
            
            THEN ("The zero function should reset it") {
                vec3.zero();
                REQUIRE(vec3.x == 0.f);
                REQUIRE(vec3.y == 0.f);
                REQUIRE(vec3.z == 0.f);
            }
        }
    }
    
    
    GIVEN ("A KudanQuaternion") {
        
        WHEN("Not initialised") {
            KudanQuaternion quat;
            THEN("All values should be zero") {
                REQUIRE(quat.w == 0.f);
                REQUIRE(quat.x == 0.f);
                REQUIRE(quat.y == 0.f);
                REQUIRE(quat.z == 0.f);
            }
        }
        
        WHEN("Setting values with constructor") {
            KudanQuaternion quat(1,2,3,4);
            THEN ("The values should be set in order w-x-y-z") {
                REQUIRE(quat.x == 1.f);
                REQUIRE(quat.y == 2.f);
                REQUIRE(quat.z == 3.f);
                REQUIRE(quat.w == 4.f);
            }
        }
        WHEN("Setting some values") {
            
            KudanQuaternion quat(9.7, 3.3, 5, 0.3);
            THEN ("The values should be reset by the zero() function") {
                quat.zero();
                REQUIRE(quat.w == 0.f);
                REQUIRE(quat.x == 0.f);
                REQUIRE(quat.y == 0.f);
                REQUIRE(quat.z == 0.f);
            }
        }
    }
    
    GIVEN ("A KudanMatrix3") {
        
        KudanMatrix3 m3;
        WHEN ("Not initialised") {
            THEN ("All values should be zero") {
                for (int i = 0; i < 9; i++) {
                    REQUIRE(m3.data[i] == 0.f);
                }
            }
            THEN ("All values should be zero via ()") {
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        REQUIRE(m3(r,c) == 0.f);
                    }
                }
            }
        }
        
        WHEN ("Setting values with ()") {
            // test by assigning each element a uniqye value as a function of row, col
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m3(r,c) = 42*c + 0.4567f*r;
                }
            }
            THEN ("Should be able to access the same values with () ") {
                
                // test by assigning each element a uniqye value as a function of row, col
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        REQUIRE(m3(r,c) == 42*c + 0.4567f*r);
                    }
                }
            }
        }
    }

    
    GIVEN( "A KudanMatrix4" ) {
        
        KudanMatrix4 m4;
        WHEN ("Not initialised") {
            THEN ("All values should be zero") {
                for (int i = 0; i < 16; i++) {
                    REQUIRE(m4.data[i] == 0.f);
                }
            }
            THEN ("All values should be zero via ()") {
                for (int r = 0; r < 4; r++) {
                    for (int c = 0; c < 4; c++) {
                        REQUIRE(m4(r,c) == 0.f);
                    }
                }
            }
        }
        
        WHEN ("Setting values with ()") {
            // test by assigning each element a uniqye value as a function of row, col
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    m4(r,c) = 12*r + 1.9876f*c;
                }
            }
            THEN ("Should be able to access the same values with () ") {
                
                // test by assigning each element a uniqye value as a function of row, col
                for (int r = 0; r < 4; r++) {
                    for (int c = 0; c < 4; c++) {
                        REQUIRE(m4(r,c) == 12*r + 1.9876f*c);
                    }
                }
            }
        }
    }
    
    // Maths operations
    
    GIVEN ("A KudanMatrix3 and a KudanVector3") {
        
        int numTests = 100;
        WHEN ("They are both zero") {
            KudanMatrix3 M;
            KudanVector3 V;
            THEN ("Their product is zero") {
                KudanVector3 MV = M.multiply(V);
                REQUIRE(MV.x == 0.f);
                REQUIRE(MV.y == 0.f);
                REQUIRE(MV.z == 0.f);
            }
        }
        
        WHEN ("Multiplying by identity matrix") {
            KudanMatrix3 I;
            // Build identity:
            I(0,0) = I(1,1) = I(2,2) = 1.f;
            
            int vecRange = 1000;
            // test lots of random vectors!
            
            THEN ("Any vector is unchanged") {
                for (int i = 0; i < numTests; i++) {
                
                    KudanVector3 V(rand()%vecRange, rand()%vecRange, rand()%vecRange);
                    KudanVector3 IV = I.multiply(V);
                    REQUIRE(V.x == IV.x);
                    REQUIRE(V.y == IV.y);
                    REQUIRE(V.z == IV.z);
                    
                }
            }
        }
        WHEN ("Multiplying a vector by some matrix") {
            
            KudanMatrix3 M;
            // sequeintial element matrix
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    M(r,c) = c*3 + r;
                }
            }
            // sequential data vector
            KudanVector3 V(1,2,3);
            
            /* multiplication should do this:
            0 3 6 * 1   =  0 + 6 + 18  = 24
            1 4 7   2      1 + 8 + 21    30
            2 5 8   3      2 + 10 + 24   36
             */
            
            THEN ("Result should be as expected ") {
                KudanVector3 MV = M.multiply(V);
                REQUIRE(MV.x == 24.f);
                REQUIRE(MV.y == 30.f);
                REQUIRE(MV.z == 36.f);
            }
            
        }

        WHEN ("Adding two vectors") {
            
            int vecRange = 1000;
            
            THEN ("Result should always be correct, and symmetric") {
                for (int i = 0; i < numTests; i++) {
                 
                    float a0 = rand()*vecRange - vecRange/2;
                    float a1 = rand()*vecRange - vecRange/2;
                    float a2 = rand()*vecRange - vecRange/2;
                    float b0 = rand()*vecRange - vecRange/2;
                    float b1 = rand()*vecRange - vecRange/2;
                    float b2 = rand()*vecRange - vecRange/2;
                    KudanVector3 A(a0,a1,a2);
                    KudanVector3 B(b0,b1,b2);
                    
                    KudanVector3 ApB = A.add(B);
                    KudanVector3 BpA = B.add(A);
                    
                    REQUIRE(ApB.x == a0+b0);
                    REQUIRE(ApB.y == a1+b1);
                    REQUIRE(ApB.z == a2+b2);
                    
                    REQUIRE(BpA.x == a0+b0);
                    REQUIRE(BpA.y == a1+b1);
                    REQUIRE(BpA.z == a2+b2);
                    
                    
                }
            }
        }

        
        
    }
    
    
    
}

