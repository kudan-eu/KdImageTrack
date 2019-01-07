# KudanImageTrack

The Kudan C++ library for the detection and tracking of markers and markerless scenes.

## Getting Started

These instructions will get a copy of the project up and running on your local machine for development purposes.

### Prerequisites

#### iOS / macOS

XCode 8.2.1 or later.

#### Android

Android NDK toolset r13 or later.

### Installing and Building

Clone the repository.

#### iOS / macOS

Open `KudanImageTrack.xcodeproj` in Xcode, located in the root directory.

Select the `KudanImageTrack-iOS` or `KudanImageTrack-macOS` schemes and build for the compiled static libraries.

Various demo schemes also exist to give examples of the static library's usage.


#### Android

In Terminal, navigate to the `Android` directory.

```
cd $(KUDANIMAGETRACK_PROJECT_DIR)/Android
```

Run `ndk-build`.

```
$(ANDROID_SDK)/sdk/ndk-bundle/ndk-build
```

The `.so` shared libraries will be build into the `libs` folder.

## Running tests

Select and build the `IntegrationTests` scheme in Xcode.
