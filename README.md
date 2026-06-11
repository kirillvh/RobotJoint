\## Efficient C++ Library for Humanoid Robotics

* Forward Kinematics
* RNE Dynamics including Gravity Compensation
* Jacobian, COM Jacobian and its inverse



## Dependencies and Credits

* Matrix Library: This library uses the Eigen matrix library (http://eigen.tuxfamily.org)
* Path Planning: This library uses a MODIFIED version of the "MicroPather" code originally developed by Lee Thomason(micropather.h \&\& micropather.cxx). The original version can be found at http://grinninglizard.com/MicroPather/ and http://sourceforge.net/projects/micropather/
* CPU specific optimization(i.e SSE) cmake script: This library uses some cmake optimization scripts(FindSSE.cmake, OptimizeForArchitecture.cmake, AddCompilerFlag.cmake?, AddTargetProperty.cmake>) to enable cpu specific optimizations like SSE. These scripts were originally developed by Matthias Kretz  and can be found at http://gitorious.org/vc/vc/trees/master/cmake. They also use some BSD licensed scripts by different authors (MacroEnsureVersion.cmake, CheckCXXCompilerFlag.cmake, CheckCCompilerFlag.cmake).
* Linear Algebra routines: This library makes use of LAPACK for some of the linear algebra routines not yet provided in Eigen.
* Documentation: This library uses DOXYGEN to generate documentation from source code. Credits to Stefan Majewsky from the "The Geek Shall Inherit The Earth" blog (https://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen) for the CMake script that integrates DOXYGEN with CMake



## License

RobotJoint is licensed under the [MIT License](LICENSE). Bundled third-party
components retain their original licenses.

