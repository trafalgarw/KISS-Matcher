<div align="center">
    <h1>KISS-Matcher</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/763bafef-c11a-4412-a9f7-f138fc12ff9f" alt="KISS Matcher" width="80%"/></p>
  <p><strong><em>Keep it simple, make it scalable.</em></strong></p>
</div>

---

[YouTubeLInk]: https://www.youtube.com/watch?v=fogCM159GRk
[arXivlink]: https://arxiv.org/abs/2207.11919

---

## :package: Prerequisite packages

> First, what we need are just minimal dependencies.

```commandline
sudo apt-get install gcc g++ build-essential libeigen3-dev cmake python3-pip python3-dev git ninja-build libflann-dev -y
```


## :gear: How to build & Run



### C++

As a dependency, we need [ROBIN](https://github.com/MIT-SPARK/ROBIN).
If your Ubuntu version is equal or higher than 22.04, you might encounter a MINSIGSTKSZ error during the build process.
If so, please follow [these instructions](https://github.com/MIT-SPARK/ROBIN/issues/2).


After then, just run

```commandline
make cppinstall
```

To call our KISS-Matcher package to another package, please refer to below example:



```cmake
# Step 1. set find package
find_package(kiss_matcher REQUIRED)
...

# below code is an example
add_executable(main_benchmark src/benchmark.cpp
        )
target_include_directories(main_benchmark
        PUBLIC
        ${HDF5_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${KISS_ICP_DIRS}
        )
target_link_libraries(main_benchmark
        PUBLIC
        ${OpenCV_LIBRARIES}
        ${HDF5_LIBRARIES}
        ${PCL_LIBRARY_DIRS}
        ${catkin_LIBRARIES}
        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io teaserpp::teaser_features
        kiss_matcher::kiss_matcher_core #  <- Like this, you need to set link the library
        stdc++fs
        glog
        )
```

### Python

TBU.
---
