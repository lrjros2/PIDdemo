# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lirj25/ctrlDemo/PIDdemo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lirj25/ctrlDemo/PIDdemo/build

# Include any dependencies generated for this target.
include CMakeFiles/demo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/demo.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo.dir/flags.make

CMakeFiles/demo.dir/main.cc.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/main.cc.o: ../main.cc
CMakeFiles/demo.dir/main.cc.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lirj25/ctrlDemo/PIDdemo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/demo.dir/main.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/demo.dir/main.cc.o -MF CMakeFiles/demo.dir/main.cc.o.d -o CMakeFiles/demo.dir/main.cc.o -c /home/lirj25/ctrlDemo/PIDdemo/main.cc

CMakeFiles/demo.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo.dir/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lirj25/ctrlDemo/PIDdemo/main.cc > CMakeFiles/demo.dir/main.cc.i

CMakeFiles/demo.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo.dir/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lirj25/ctrlDemo/PIDdemo/main.cc -o CMakeFiles/demo.dir/main.cc.s

CMakeFiles/demo.dir/pid.cc.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/pid.cc.o: ../pid.cc
CMakeFiles/demo.dir/pid.cc.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lirj25/ctrlDemo/PIDdemo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/demo.dir/pid.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/demo.dir/pid.cc.o -MF CMakeFiles/demo.dir/pid.cc.o.d -o CMakeFiles/demo.dir/pid.cc.o -c /home/lirj25/ctrlDemo/PIDdemo/pid.cc

CMakeFiles/demo.dir/pid.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo.dir/pid.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lirj25/ctrlDemo/PIDdemo/pid.cc > CMakeFiles/demo.dir/pid.cc.i

CMakeFiles/demo.dir/pid.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo.dir/pid.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lirj25/ctrlDemo/PIDdemo/pid.cc -o CMakeFiles/demo.dir/pid.cc.s

# Object files for target demo
demo_OBJECTS = \
"CMakeFiles/demo.dir/main.cc.o" \
"CMakeFiles/demo.dir/pid.cc.o"

# External object files for target demo
demo_EXTERNAL_OBJECTS =

../bin/demo: CMakeFiles/demo.dir/main.cc.o
../bin/demo: CMakeFiles/demo.dir/pid.cc.o
../bin/demo: CMakeFiles/demo.dir/build.make
../bin/demo: /usr/lib/x86_64-linux-gnu/libpython3.10.so
../bin/demo: CMakeFiles/demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lirj25/ctrlDemo/PIDdemo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo.dir/build: ../bin/demo
.PHONY : CMakeFiles/demo.dir/build

CMakeFiles/demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo.dir/clean

CMakeFiles/demo.dir/depend:
	cd /home/lirj25/ctrlDemo/PIDdemo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lirj25/ctrlDemo/PIDdemo /home/lirj25/ctrlDemo/PIDdemo /home/lirj25/ctrlDemo/PIDdemo/build /home/lirj25/ctrlDemo/PIDdemo/build /home/lirj25/ctrlDemo/PIDdemo/build/CMakeFiles/demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo.dir/depend

