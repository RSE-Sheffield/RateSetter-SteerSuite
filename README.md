SteerSuite
==========

Welcome!  SteerSuite is a set of test cases, tools, and a C++ library for developing and evaluating agent steering AI behaviors. This is the RateSetter implementation of SteerSuite, tailored to Platform-Train-Interactions (PTI), bags, and group pedestrian behaviour.

Enjoy!

Documentation
-------------

If you want to build your own documentation from this package, read the file `documentation/readme.txt` for more information.  Otherwise, the latest documentation for the original/Upstream version of SteerSuite can be found at the [SteerSuite web page](http://steersuite.eecs.yorku.ca/).

The documentation includes:

  - The User Guide explains how to use most features of SteerSuite.
  - The Reference Manual contains a comprehensive listing of
    SteerSuite components.
  - The Code Reference is doxygen-generated documentation of the C++
    code.

Directory structure
-------------------

The directory structure of this package is as follows:

    documentation   - raw unprocessed documentation and instructions for
                      building the documentation.

    external        - external dependencies that are (legally) included
                      for convenience, but NOT part of SteerSuite.

    kdtree          - Spatial Database module for a kdtree type spatial
                      databse.

    navmeshBuilder  - Module to build navigation meshes during runtime.
                      Uses Recast.

    rvo2AI          - source directory for the ORCA steering module,
                      based on the RVO2 steering algorithm library.

    steerbench      - source directory for SteerBench, a tool used to
                      score and analyze steering AI.

    steerlib        - source directory for SteerLib, a shared library
                      containing most of SteerSuite's functionality.

    steersim        - source directory for SteerSim, a modular
                      simulation tool.

    steertool       - source directory for SteerTool, a utility for
                      useful miscellaneous tasks.

    testcases       - XML test cases and the XML schema describing the
                      test case format.

Compiling SteerSuite
--------------------

Below are instructions for compiling with default options. The compiling process has changed drastically from the original Steersuite, and more information on the original implementation can be found at [SteerSuite User Guide](http://steersuite.eecs.yorku.ca/UserGuide/).

Steersuite has several dependencies:

+ [CMake](http://cmake.org/) as a cross platform build system generator
+ C++ compiler for your platform (i.e. gcc/g++, clang/clang++, MSVC) and the platform specific build system (Make, Visual Studio, Xcode).
+ Optionally graphics libraries if interactive visualisations are required, including OpenGL and GLUT/FreeGLUT (and possibly GLEW).

For example, on Ubuntu 20.04 the following should be sufficient
```bash
    apt-get install cmake build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev libglew1.6-dev mesa-utils libglew-dev freeglut3-dev  
```
On Windows you will need to download and install CMake and an appropriate version of Visual Studio, with the appropriate Graphics drivers installed for yout system.
If GLUT or FreeGLUT cannot be found by CMake, a copy of FreeGLUT 3 will be downloaded during CMake configuration and it will be compiled at build time, otherwise a system-wide install of glut/freeglut will be used instead. 
`vcpkg` can be use 

Executable files will be placed within the `build/bin/<config>` directory (where `build` is the chosen out of tree build directory). 
`.so` and `.lib` will be in `build/lib/<config>`. 


### CMake GUI

+ Open the CMake GUI
+ Select the root of the repository as the source directory
+ Create and select the `build` directory within the source directory to build artifacts into
+ Press `Configure`, choosing the desired Generator for your platform
  + On Linux select `Unix Makefiles`
  + On Windows select the appropriate Visual Studio Version
  + On Macos **@todo**
+ Change any options as required. 
  + I.e. uncheck `ENABLE_GUI` if you do not want to build with visualiastion enabled
+ Press `Generate`
+ You can then build through your chosen IDE, or navigate to the `build` directory and execute `cmake --build .`

### CMake CLI

#### Linux 

```bash
mkdir -p build 
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j `nproc`
```

#### Windows

```powershell
mkdir -p build
cd build
# Generate 64-bit applications using for Visual Studio 2019 
cmake .. -A x64 -G "Visual Studio 16 2019"
# Open Visual Studio
cmake --open . 
# Or build from the command line 
cmake --build . --config Release --target ALL_BUILD
```

#### macOS

> Note macOS instructions are untested

```bash
mkdir -p build 
cd build
cmake .. 
# Open Xcode
cmake --open .
# Or build from teh command line
cmake --build . --config Release --target all
```

### CMake Build Options

#### Build Configuration

Makefile-like Cmake generators select the build config at Cmake Configuraiton time, using `CMAKE_BUILD_TYPE`. 

```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

Multi-target generators such as Visual Studio and xcode handle this as usual, however if `cmake --build` is being used in place of the IDE, the `--config` parameter can be used.

```bash
cmake --build . --config Debug 
```

`Release` and `Debug` are the only configurations which are likely to be required, although `RelWithDebInfo` (optimised debug) and `MinSizeRel` (optimised for file size not speed) are also available.

#### GUI

Steersim/Steerbench can be used with GLFW based visualisation, or in batch mode without visualisation via the `-commandline` flag. 
To use of batch mode in locations where OpenGL dependencies may not be available (such as regular nodes in HPC systems), cmake can be configured without the GUI enabled, by setting `ENABLE_GUI=OFF` either via the command line or gui. I.e.

```bash
cmake .. -DENABLE_GUI=OFF
```
the `-commmandline` argument must still be passed when running the application in batch mode.


#### Compiler Warnings

The previous premake-based build system justifiably set high warning levels, however this led to thousands of warnings being emitted and so compiler warning levels have been set very low by default.
A higher level of warnings can be enables by setting `ENABLE_WARNINGS=ON` via the cmake commandline or GUI. This is not currently recommended unless you are intending to fix the warnings.


Debuggging Steersim
-------------------

When built using the `Debug` config (or `RelWithDebInfo` to a certain extent), source level debugging can be achieved through Visual Studio, or `gdb` depending on platform. This support may also be available other IDEs/editors such as Visual Studio Code.

### Visual Studio

+ Set the `steersim` or `steerbench` project as the `Startup Project` depending on which you wish to debug
  + Right click the project 
  + Select `Set as StartUp Project` 
+ Set the `Command Arguments` for the project:
  + Right click the project
  + Select `Properties`
  + Ensure the correct confiuguration is selected (All or `Debug`)
  + Select the `Debugging Section`
  + Populate the `Command Arguments` option

### Visual Studio Code (linux)

Under linux (and presumably other OS) Visual Studio Code can be used to graphically debug c++ applications, by creating the file `.vscode/launch.json`, and adjusting it with the appropraite paths.
This requires the `ms-vscode.cpptools` plugin and optionally `ms-vscode.cmake` plugin.

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/bin/Debug/steersim",
            "args": ["-config", "../testcases/Merseyrail/myconfig.xml"],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}/build",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```


If you are using the CMake vscode plugin, then the following values can be used instead for a more flexible confiugration.

```json
// ...
{
  "program": "${command:cmake.launchTargetPath}",
  "cwd": "${command:cmake.buildDirectory}",
}
```

### GDB (linux)

`gdb` can be used for command line debugging, consider `--args` option.
As steersim loads shard objects at runtime, setting breakpoints in objects loaded at runtime will generate warnings, and may require setting additional options for source files to be successfully located.

```
cd build


```

Contact information
-------------------

Contact Information:

- John Charlton     `j.a.charlton@sheffield.ac.uk`
- Peter Heywood     `p.heywood@sheffield.ac.uk`
- Paul Richmond     `p.richmond@sheffield.ac.uk `

Please report bugs by opening an issue on
[github](https://github.com/RSE-Sheffield/RateSetter-SteerSuite/issues).

Original SteerSuite
---------------------

The original SteerSuite implementation web page is [steersuite.eecs.yorku.ca](http://steersuite.eecs.yorku.ca/), and the repository is on [github.com/SteerSuite/Release](https://github.com/SteerSuite/Release)

Original SteerSuite web page:
[steersuite.eecs.yorku.ca](http://steersuite.eecs.yorku.ca/)

Original SteerSuite Forum (Google Group):
[groups.google.com/forum/#!forum/steersuite](https://groups.google.com/forum/#!forum/steersuite)

Copyright and license
---------------------

Original SteerSuite, SteerBench, SteerBug, SteerSim, and SteerLib are Copyright
(c) 2008-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros
Faloutos, and Glenn Reinman.

See `license.txt` for the complete license.

NOTE: The contents of the `external/` directory are NOT part of
SteerSuite.  Each component in `external/` has its own authors,
copyright, and license, and those souces are only included for
convenience.

Credits
-------

Refer to the [SteerSuite web page](http://steersuite.eecs.yorku.ca/) for credits and acknowledgements.
