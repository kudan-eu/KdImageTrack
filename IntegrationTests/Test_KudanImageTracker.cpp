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
#include "catch.hpp"
#include "KdImageTrack.h"
#include "UnitTestUtils.hpp"



TEST_CASE( "Kudan initialisation checks", "[Tracker]" ) {
    

    GIVEN( "A KdImageTracker" ) {
        KdImageTracker tracker;
        
        WHEN ("There is no calibration") {
            
            THEN ("it knows it has no calibration") {
                REQUIRE( tracker.hasCameraCalibration() == false);
            }
            
        }
        
        WHEN ("There is a calibration") {

            KudanCameraParameters cameraParameters;
            cameraParameters.setSize(640,360);
            cameraParameters.guessIntrinsics();
            
            tracker.setCameraParameters(cameraParameters);
            
            THEN ("it knows it has a calibration") {
                REQUIRE( tracker.hasCameraCalibration() == true);

            }
        }
        
        
        WHEN ("An invalid calibtation is used") {
            
            
            KudanCameraParameters cameraParameters;
            THEN ("The calibration is invalid") {
                REQUIRE( cameraParameters.isValid() == false);
            }
            THEN ("An exteption is thrown ") {
                REQUIRE_THROWS_AS( tracker.setCameraParameters(cameraParameters), KudanException);
            }
            
        }
        
        
        WHEN ("An nonexistent marker is added") {
            THEN ("An exteption is thrown for \"\" ") {
                REQUIRE_THROWS_AS( tracker.addTrackable(""), KudanException);
            }
            THEN ("An exteption is thrown for \"/usr/local/bad_path\"") {
                REQUIRE_THROWS_AS( tracker.addTrackable("/usr/local/bad_path"), KudanException);
            }
        }
        
        GIVEN( "The data folder" ) {
            
            std::string dataFolder = "./IntegrationTests/TestData";
            
            REQUIRE(dataFolder != "");
            
            WHEN ("An good marker is added (lego)") {
            
                THEN ("An exteption is not thrown") {
                    REQUIRE_NOTHROW( tracker.addTrackable(dataFolder + "/lego.jpg") );
                }
            }
        
        
            WHEN ("An bad marker is added (blank image)") {

                THEN ("An exteption is not thrown") {

                    REQUIRE_NOTHROW( tracker.addTrackable(dataFolder + "/blank.jpg"));
                }
                THEN ("Return false") {
                    REQUIRE( tracker.addTrackable(dataFolder + "/blank.jpg") == false);
                }
            }
        }
    }
    
    
    
    
}

