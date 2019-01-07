//
//  Test_KdImageTracker.cpp
//  KdImageTrack
//
//  Created on 20/09/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

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

