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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

TEST_CASE( "KdImageTrackable checks", "[Trackable]" ) {

    std::string dataFolder = "./IntegrationTests/TestData";
    
    GIVEN("A folder of data = " + dataFolder) {
        REQUIRE(dataFolder != "");
        
       
        
        WHEN ("Creating a KdImageTrackable from file") {
            
            WHEN("When creating from a valid jpg (lego)") {
                
                std::string legoFile = dataFolder + "/lego.jpg";
                THEN ("No exception") {
                    REQUIRE_NOTHROW(std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(legoFile));
                }
                THEN ("Result not null") {
                    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(legoFile);
                    REQUIRE(trackable != nullptr);
                }
                
            }
            WHEN("When creating from valid png") {
                
                std::string starsFile = dataFolder + "/stars320.png";
                THEN ("No exception") {
                    REQUIRE_NOTHROW(std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(starsFile));
                }
                THEN ("Result not null") {
                    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(starsFile);
                    REQUIRE(trackable != nullptr);
                }
            }
            WHEN("When creating from a blank white image file") {
                
                std::string blankFile = dataFolder + "/blank.jpg";
                THEN ("No exception") {
                    REQUIRE_NOTHROW(std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(blankFile));
                }
                THEN ("Result IS null") {
                    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(blankFile);
                    REQUIRE(trackable == nullptr);
                }
                
            }
            
            if (doSlowTests) {
                WHEN("When creating from a very small jpg (Mindmaze)") {
                    std::string mindmazeFile = dataFolder + "/MM_poster320.jpg";
                    THEN ("No exception") {
                        REQUIRE_NOTHROW(std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(mindmazeFile));
                    }
                    THEN ("Result not null") {
                        std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(mindmazeFile);
                        REQUIRE(trackable != nullptr);
                    }
                }
                WHEN("When creating from a very large image (Formula E)") {
                    std::string plinthFile = dataFolder + "/Plinth.png";
                    THEN ("No exception") {
                        REQUIRE_NOTHROW(std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(plinthFile));
                    }
                    THEN ("Result not null") {
                        std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(plinthFile);
                        REQUIRE(trackable != nullptr);
                    }
                }
            }
            GIVEN ("The lego file") {
                
                std::string legoFile = dataFolder + "/lego.jpg";

                WHEN("Deriving the name") {
                    
                    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(legoFile);
                    REQUIRE(trackable != nullptr);
                    THEN ("Name is equal to the stripped file name") {
                        REQUIRE(trackable->getName() == "lego");
                    }
                }
                WHEN("Setting the name") {
                    
                    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(legoFile, "MarkerName");
                    REQUIRE(trackable != nullptr);
                    THEN ("Name is NOT equal to the stripped file name") {
                        REQUIRE(trackable->getName() != "lego");
                    }
                    THEN ("Name is equal to the specified name") {
                        REQUIRE(trackable->getName() == "MarkerName");
                    }
                }
            }


            
        }
        WHEN ("Creating a KdImageTrackable from image data") {
            
            
            
            WHEN("Creating from null data") {
                THEN ("Throw an exception when all arguments are 0") {
                    REQUIRE_THROWS_AS( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(0, "lego", 0, 0, 0, 0) , KudanException);
                }
                THEN ("Throw an exception when only data arguments is null") {
                    REQUIRE_THROWS_AS( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(0, "lego", 640, 480, 3, 0) , KudanException);
                }
            }
            
            if (doSlowTests) {
                GIVEN ("The lego file") {
                    
                    std::string legoFile = dataFolder + "/lego.jpg";
                    GIVEN ("The lego image ") {
                        cv::Mat legoImage = cv::imread(legoFile);
                        REQUIRE(legoImage.rows > 0);
                        REQUIRE(legoImage.cols > 0);
                        REQUIRE(legoImage.channels() == 3);
                        
                        WHEN("Creating from colour image data") {
                            THEN ("No exception") {
                                REQUIRE_NOTHROW( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(legoImage.data, "lego", legoImage.size().width, legoImage.size().height, legoImage.channels(), 0) );
                            }
                            THEN ("Is not null") {
                                
                                std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(legoImage.data, "lego", legoImage.size().width, legoImage.size().height, legoImage.channels(), 0);
                                REQUIRE(trackable != nullptr);
                                
                            }
                        }
                        
                        WHEN("Creating from grey image data") {

                            cv::Mat greyImage;
                            cv::cvtColor(legoImage, greyImage, CV_BGR2GRAY);
                            
                            REQUIRE(greyImage.channels() == 1);
                            
                            THEN ("No exception") {
                                REQUIRE_NOTHROW( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(greyImage.data, "lego", greyImage.size().width, greyImage.size().height, greyImage.channels(), 0) );
                            }
                            THEN ("Is not null") {
                                
                                std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(greyImage.data, "lego", greyImage.size().width, greyImage.size().height, greyImage.channels(), 0);
                                REQUIRE(trackable != nullptr);
                                
                            }
                            
                        }
                        
                        WHEN("Creating from colour+alpha image data") {
                            
                            cv::Mat rgbaImage;
                            cv::cvtColor(legoImage, rgbaImage, CV_BGR2BGRA);
                            
                            REQUIRE(rgbaImage.channels() == 4);
                            
                            THEN ("No exception") {
                                REQUIRE_NOTHROW( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(rgbaImage.data, "lego", rgbaImage.size().width, rgbaImage.size().height, rgbaImage.channels(), 0) );
                            }
                            THEN ("Is not null") {
                                
                                std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(rgbaImage.data, "lego", rgbaImage.size().width, rgbaImage.size().height, rgbaImage.channels(), 0);
                                REQUIRE(trackable != nullptr);
                                
                            }
                            
                        }

                        
                        
                        WHEN("Setting wrong number of channels") {
                            THEN ("No exception") {
                                REQUIRE_NOTHROW( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(legoImage.data, "lego", legoImage.size().width, legoImage.size().height, 2, 0) );
                            }
                            THEN ("Is null") {
                                
                                std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(legoImage.data, "lego", legoImage.size().width, legoImage.size().height, 2, 0);
                                REQUIRE(trackable == nullptr);
                                
                            }
                        }
                        
                        WHEN("Setting no channels") {
                            THEN ("Throws exception") {
                                REQUIRE_THROWS_AS( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(legoImage.data, "lego", legoImage.size().width, legoImage.size().height, 0, 0), KudanException );
                            }
                        }
                        WHEN("Setting too many channels") {
                            THEN ("Throws exception") {
                                REQUIRE_THROWS_AS( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(legoImage.data, "lego", legoImage.size().width, legoImage.size().height, 5, 0), KudanException );
                            }
                        }
                        

                        
                    }
                }
                
                GIVEN ("The lego marker") {
                    std::string legoFile = dataFolder + "/lego.jpg";

                    cv::Mat legoImage = cv::imread(legoFile);
                    
                    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageData(legoImage.data, "lego", legoImage.size().width, legoImage.size().height, legoImage.channels(), 0);

                    REQUIRE(trackable != nullptr);
                    
                    
                    WHEN ("Size is set to zero") {
                        THEN("An exception is thrown") {
                            REQUIRE_THROWS_AS(trackable->setSize(0, 0), KudanException);
                        }
                    }
                    
                    WHEN ("Size is set to the wrong aspect ratio") {
                     
                        THEN ("No exception") {
                            REQUIRE_NOTHROW(trackable->setSize(4000, 100));
                        }
                        
                        THEN ("Function returns false") {
                            REQUIRE(false == trackable->setSize(4000, 100));
                        }
                    }
                    
                    WHEN ("Size is set to the original size") {
                        
                        THEN ("No exception") {
                            REQUIRE_NOTHROW(trackable->setSize(legoImage.size().width, legoImage.size().height));
                        }
                    }
                    WHEN ("Size is set to some multiple of the original size") {
                        
                        THEN ("Function returns true") {

                            float scale = 2.f;
                            REQUIRE(true == trackable->setSize(legoImage.size().width * scale, legoImage.size().height * scale));
                            
                            scale = 4.f;
                            REQUIRE(true == trackable->setSize(legoImage.size().width * scale, legoImage.size().height * scale));
                            
                            scale = 54.f;
                            REQUIRE(true == trackable->setSize(legoImage.size().width * scale, legoImage.size().height * scale));
                            
                            scale = 54.f;
                            REQUIRE(true == trackable->setSize(legoImage.size().width * scale, legoImage.size().height * scale));
                            
                        }
                    }
                    WHEN ("Size is set to something very close to the correct aspect ratio") {
                        REQUIRE(true == trackable->setSize(legoImage.size().width * 2.f, legoImage.size().height * 2.f + 2));

                    }
                    
                }
            }
        }
        
        WHEN ("Creating a KdImageTrackable from file pointer") {
            

            if (doSlowTests) {
                GIVEN ("A file pointer ") {
                    std::string legoMarkerFile = dataFolder + "/lego.KARMarker";

                    FILE* filePointer = fopen(legoMarkerFile.c_str(), "rb");
                    REQUIRE(filePointer != 0);
                    
                    THEN ("No exception") {
                        REQUIRE_NOTHROW( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromFilePointer(filePointer) );
                    }
                    THEN ("Is not null") {
                        
                        std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromFilePointer(filePointer);
                        REQUIRE(trackable != nullptr);
                        
                    }
                    
                    
                }
            }
            
            WHEN ("Creating a KdImageTrackable from null file pointer") {

                FILE *nullFile = 0;

            
                
                THEN ("No exception") {
                    REQUIRE_NOTHROW( std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromFilePointer(nullFile) );
                }
                THEN ("Is null") {
                    
                    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromFilePointer(nullFile);
                    REQUIRE(trackable == nullptr);
                    
                }
                
            }
        }
    }


}
