sot-motion-planner
==================

[![Build Status](https://travis-ci.org/stack-of-tasks/sot-motion-planner.png?branch=master)](https://travis-ci.org/stack-of-tasks/sot-motion-planner)
[![Coverage Status](https://coveralls.io/repos/stack-of-tasks/sot-motion-planner/badge.png)](https://coveralls.io/r/stack-of-tasks/sot-motion-planner)

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.

Here is an advanced cmake example:
```
ccmake .. -DCMAKE_INSTALL_PREFIX=/home/ostasse/devel/ros-unstable-1/install -DCMAKE_BUILD_TYPE=RELEASE -DPYTHON_LIBRARY=/usr/lib/libpython2.6.a -DPYTHON_INCLUDE_DIR=/usr/include/python2.6
```
which specifies the install directory, the type of build, the include directoy for python, and the associated library.


### Dependencies

The matrix abstract layer depends on several packages which
have to be available on your machine.

 - Libraries:
   - [dynamic-graph][dynamic-graph]
   - [sot-core][sot-core]
   - [jrl-mal][jrl-mal]
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


[dynamic-graph]: http://github.com/stack-of-tasks/dynamic-graph
[jrl-mal]: http://github.com/jrl-umi3218/jrl-mal
[sot-core]: http://github.com/stack-of-tasks/sot-core
