SteerSuite
==========

Welcome!  SteerSuite is a set of test cases, tools, and a C++ library
for developing and evaluating agent steering AI behaviors.

The SteerSuite web page is
[steersuite.eecs.yorku.ca](http://steersuite.eecs.yorku.ca/)

On the web page you can find a description of SteerSuite, download the
latest version of SteerSuite, read the latest documentation, and join
the discussion group.

Please send us any comments and feedback about how to make SteerSuite
easier to use.  Your input is greatly appreciated.

Enjoy!

Documentation
-------------

If you want to build your own documentation from this package, read
the file `documentation/readme.txt` for more information.  Otherwise,
the latest documentation can be found at the
[SteerSuite web page](http://steersuite.eecs.yorku.ca/).

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

    pprAI           - source directory for the PPR steering module, a
                      demo steering algorithm for SteerSim.

    rvo2AI          - source directory for the ORCA steering module,
                      based on the RVO2 steering algorithm library.

    reactiveAI      - source directory for the reactive steering module,
                      a demo steering algorithm for SteerSim (just the
                      reactive appraoch of PPR).

    socialForcesAI  - source directory for the social foces steering
                      module, an implementation of the social forces
                      steering algorithm.

    simpleAI        - source directory for the simpleAI module, a basic
                      demo plugin for SteerSim.

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

Below are quick instructions for compiling with default options. For
more complete instructions, refer to the
[SteerSuite User Guide](http://steersuite.eecs.yorku.ca/UserGuide/).

As with any graphics library you will need to make sure you already
have the OpenGL libraries on your computer. For example on
Ubuntu 14.04 you will want to install the following:

    freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev libglew1.6-dev mesa-utils libglew-dev premake4

This will install OpenGL and GLEW.

On Windows you will need to download glut32.lib and the opengl header files and put them in your Visual Studio library path.

Alternatively use `vcpkg` to install the required dependencies. The vcpkg toolchain must then be passed to CMake. 

Note: The build system has been updated and now uses
[CMake](http://cmake.org/). You are going to need to have
this installed to be able to build the software.

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

```
mkdir -p build 
cd build
cmake .. -DENABLE_GUI=ON -DCMAKE_BUILD_TYPE=Release
cmake --build . -j `nproc`
```

#### Windows

```
@todo - Similar to above but with powershell. 
-A x64 to restrict to 64bit only 
-G "Visual Studio 2019 16" or similar to select version.
Build type is handled within Visual studio.
```

#### macOs 

```
@todo.
```

### CMake Build Options

#### Build Configuration

Makefile-like Cmake generators select the build config at Cmake Configuraiton time, using `CMAKE_BUILD_TYPE`. 

```
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

Multi-target generators such as Visual Studio and xcode handle this as usual, however if `cmake --build` is being used in place of the IDE, the `--config` parameter can be used.

```
cd build && cmake --build . --config Debug 
```

`Release` and `Debug` are the only configurations which are likely to be required, although `RelWithDebInfo` (optimised debug) and `MinSizeRel` (optimised for file size not speed) are also available.

#### GUI

Steersim/Steerbench can be used with GLFW based visualisation, or in batch mode without visualisation via the `-commandline` flag. 
To use of batch mode in locations where OpenGL dependencies may not be available (such as regular nodes in HPC systems), cmake can be configured without the GUI enabled, by setting `ENABLE_GUI=OFF` either via the command line or gui. I.e.

```
cmake .. -DENABLE_GUI=OFF
```
the `-commmandline` argument must still be passed when running the application in batch mode.


#### Compiler Warnings

The previous premake-based build system justifiably set high warning levels, however this led to thousands of warnings being emitted and so compiler warning levels have been set very low by default.
A higher level of warnings can be enables by setting `ENABLE_WARNINGS=ON` via the cmake commandline or GUI. This is not currently recommended unless you are intending to fix the warnings.


Debuggging Steersim
-------------------

When built using the `Debug` config (or `RelWithDebInfo` to a certain extent), source level debugging can be achieved through Visual Studio, or `gdb` depending on platform. This support may also be available other IDEs/editors such as visual studio code.

### Visual Studio

@todo - instructions for debugging.


### Visual Studio Code (linux)

Under linux (and presumably other OS) Visual Studio Code can be used to graphically debug c++ applications, by creating the file `.vscode/launch.json`, and adjusting it with teh appropraite paths.

For instance, to debug a `Debug` build, in the `build` directory, running the `Merseyrail` config located at `testcases/Merseyrail/myconfig.xml` the following may be a good starting point:

```
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
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


If you are using the CMake vscode plugin, this may be a little different/simpler.

### GDB

`gdb` can be used for command line debugging, probably by using the `--args` option.
As steersim loads shard objects at runtime, setting breakpoints in objects loaded at runtime will generate warnings, and may require setting additional options for source files to be successfully located.

Contact information
-------------------

Contact Information:

- Glen Berseth      `glenpb@cse.yorku.ca`
- Mubbasir Kapadia  `mubbasir@cs.ucla.edu`
- Petros Faloutsos  `pfal@cse.yorku.ca`
- Glenn Reinman     `reinman@cs.ucla.edu`

SteerSuite web page:
[steersuite.eecs.yorku.ca](http://steersuite.eecs.yorku.ca/)

Forum (Google Group):
[groups.google.com/forum/#!forum/steersuite](https://groups.google.com/forum/#!forum/steersuite)

Please report bugs by opening an issue on
[github](https://github.com/SteerSuite/Release/issues). For any other
queries please use the forum.

Copyright and license
---------------------

SteerSuite, SteerBench, SteerBug, SteerSim, and SteerLib are Copyright
(c) 2008-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros
Faloutos, and Glenn Reinman.

See `license.txt` for the complete license.

NOTE: The contents of the `external/` directory are NOT part of
SteerSuite.  Each component in `external/` has its own authors,
copyright, and license, and those souces are only included for
convenience.

Credits
-------

Refer to the [SteerSuite web page](http://steersuite.eecs.yorku.ca/)
for credits and acknowledgements.
