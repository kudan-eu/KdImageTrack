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

#include <stdio.h>
#include <climits>
#include "catch.hpp"
#include "KdImageTrack.h"

TEST_CASE( "KudanCameraParameters BDD", "[CameraParameters]" ) {
    
    GIVEN( "A KudanCameraParameters" ) {
        KudanCameraParameters cameraParameters;
          
        WHEN ("a size is set") {
            
            cameraParameters.setSize(640, 360);
            THEN ("it is not valid") {
                REQUIRE( cameraParameters.isValid() == false);
            }
            
        }
        
        WHEN ("both sizes are zero") {
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(0, 0) , KudanException);
            }
        }
        WHEN ("width is zero") {
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(0, 360) , KudanException);
            }
        }
        WHEN ("height is zero") {
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(640, 0) , KudanException);
            }
        }
        
        WHEN ("width and/or height is negative") {
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(-1, 0) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(0, -1) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(-1, -1) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(-640, 0) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(0, -360) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(-INT_MAX, -INT_MAX) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(-987098234098760987, 0) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(-987098234098760987, -11111198720360870234) , KudanException);
            }
        }
        
        WHEN ("width and/or height are far too big") {
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(INT_MAX, 360) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(640, INT_MAX) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(4800000000000000000, 360) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(640, 63892340762340876) , KudanException);
            }
        }
        
        WHEN ("width and/or height is slightly too big") {
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(640, 1 << 30 + 1) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(1 << 30 + 1, 360) , KudanException);
            }
            THEN ("an exception is thrown") {
                REQUIRE_THROWS_AS( cameraParameters.setSize(1 << 30 + 1, 1 << 30 + 1) , KudanException);
            }
        }
        WHEN ("width and/or height  is just small enough") {
            THEN ("no exception is thrown") {
                REQUIRE_NOTHROW( cameraParameters.setSize(1 << 30, 480));
            }
            THEN ("no exception is thrown") {
                REQUIRE_NOTHROW( cameraParameters.setSize(640, 1 << 30));
            }
            THEN ("no exception is thrown") {
                REQUIRE_NOTHROW( cameraParameters.setSize(1 << 30, 1 << 30));
            }
        }
        
       
        WHEN ("a size is set and parameters guessed") {
            
            REQUIRE_NOTHROW(cameraParameters.setSize(640, 360) );
            THEN ("no exception is thrown on guessing") {
                REQUIRE_NOTHROW(cameraParameters.guessIntrinsics());
            }
        
            THEN ("it is valid") {
                cameraParameters.guessIntrinsics();
                REQUIRE( cameraParameters.isValid() == true);
            }
        }
        WHEN ("Size is set to 640 x 480") {
            // assume this is fine because we tested it above?
            
            REQUIRE_NOTHROW( cameraParameters.setSize(640, 480) );
            REQUIRE_NOTHROW( cameraParameters.guessIntrinsics() );

            REQUIRE( cameraParameters.isValid() == true);

            THEN ("It has the pre-set focal length") {
                float focalX, focalY;
                cameraParameters.getFocalLength(focalX, focalY);
                REQUIRE( focalX == 546.38889333f);
                REQUIRE( focalY == 546.79196157f);
            }
            
            THEN ("It has the pre-set principal point") {
                float principalX, principalY;
                cameraParameters.getPrincipalPoint(principalX, principalY);
                REQUIRE( principalX == 319.38145619f);
                REQUIRE( principalY == 234.44848367f);
            }

        }
        
        WHEN ("Size is set to 640 x 360") {

            REQUIRE_NOTHROW( cameraParameters.setSize(640, 360) );
            REQUIRE_NOTHROW( cameraParameters.guessIntrinsics() );
            
            REQUIRE( cameraParameters.isValid() == true);

            
            THEN ("It has the pre-set focal length") {
                float focalX, focalY;
                cameraParameters.getFocalLength(focalX, focalY);
                REQUIRE( focalX == 485.96432317873200f);
                REQUIRE( focalY == 485.96432317873200f);
            }
            THEN ("It has the pre-set principal point") {
                float principalX, principalY;
                cameraParameters.getPrincipalPoint(principalX, principalY);
                REQUIRE( principalX == 318.5892333984375f);
                REQUIRE( principalY == 197.3907470703125f);
            }
            
        }
        
        
        WHEN ("Size is set to arbitrary values") {
           
            int numTests = 20;
            srand(0);
            // randomized test
            
            // Choose random image sizes in the range [100,1000)
            int minSize = 100;
            int maxSize = 1000;
            std::function<int()> randSize = [&]() {
                int r = rand() % (maxSize - minSize);
                return r+minSize;
            };
            
            // But there are a certain set of sizes which should not be used (because those are special)
            std::vector<std::pair<int,int>> presetSizes = { std::pair<int,int>(640,480), std::pair<int,int>(640,360) };
            
            
            for (int i = 0; i < numTests; i++) {
                int inputWidth = randSize();
                int inputHeight = randSize();
                
                // if the width,height combination happens to be a preset, then alter it very slightly
                for (auto &s : presetSizes) {
                    int w = s.first;
                    int h = s.second;
                    
                    // this works because I know none of the preset sizes differ by only one pixel!
                    if (inputWidth == w && inputHeight == h) {
                        inputWidth++;
                    }
                }
                
                GIVEN("Size is set to " + std::to_string(inputWidth) + " , " + std::to_string(inputHeight)) {
                    
                    REQUIRE_NOTHROW( cameraParameters.setSize(inputWidth, inputHeight) );
                    
                    
                    REQUIRE( cameraParameters.isValid() == false);
                    
                    THEN ("The same size is retrieved") {
                        
                        int outputWidth, outputHeight;
                        cameraParameters.getSize(outputWidth, outputHeight);
                        
                        REQUIRE(inputWidth == outputWidth);
                        REQUIRE(inputHeight == outputHeight);
                    }
                    
                    REQUIRE_NOTHROW( cameraParameters.guessIntrinsics() );
                    REQUIRE( cameraParameters.isValid() == true);

                    
                    THEN ("Size is not changed by guessing intrinsics") {
                        
                        int outputWidth, outputHeight;
                        cameraParameters.getSize(outputWidth, outputHeight);
                        
                        REQUIRE(inputWidth == outputWidth);
                        REQUIRE(inputHeight == outputHeight);
                    }

                   
                    
                    
                    THEN ("It has the correct focal length") {
                        // focal lenth is calculated as the max of the width and height
                        float targetFocalLength = std::max(inputWidth, inputHeight);
                        
                        float focalX, focalY;
                        cameraParameters.getFocalLength(focalX, focalY);
                        REQUIRE( focalX == targetFocalLength);
                        REQUIRE( focalY == targetFocalLength);
                    }
                    THEN ("It has the correct principal point") {
                        // principal point is set to the midpoint
                        float principalX, principalY;
                        cameraParameters.getPrincipalPoint(principalX, principalY);
                        REQUIRE( principalX == inputWidth/2.f);
                        REQUIRE( principalY == inputHeight/2.f);
                    }
                }
            }
            
        }
        
    }
    
    
}
