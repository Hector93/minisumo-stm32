# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hector/Documentos/Robotica/firmware/minisumo-stm32

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hector/Documentos/Robotica/firmware/minisumo-stm32/build

# Utility rule file for minisumo-stm32.hex.

# Include the progress variables for this target.
include CMakeFiles/minisumo-stm32.hex.dir/progress.make

CMakeFiles/minisumo-stm32.hex: minisumo-stm32.elf
	arm-none-eabi-objcopy -Oihex minisumo-stm32.elf minisumo-stm32.hex

minisumo-stm32.hex: CMakeFiles/minisumo-stm32.hex
minisumo-stm32.hex: CMakeFiles/minisumo-stm32.hex.dir/build.make

.PHONY : minisumo-stm32.hex

# Rule to build all files generated by this target.
CMakeFiles/minisumo-stm32.hex.dir/build: minisumo-stm32.hex

.PHONY : CMakeFiles/minisumo-stm32.hex.dir/build

CMakeFiles/minisumo-stm32.hex.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/minisumo-stm32.hex.dir/cmake_clean.cmake
.PHONY : CMakeFiles/minisumo-stm32.hex.dir/clean

CMakeFiles/minisumo-stm32.hex.dir/depend:
	cd /home/hector/Documentos/Robotica/firmware/minisumo-stm32/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hector/Documentos/Robotica/firmware/minisumo-stm32 /home/hector/Documentos/Robotica/firmware/minisumo-stm32 /home/hector/Documentos/Robotica/firmware/minisumo-stm32/build /home/hector/Documentos/Robotica/firmware/minisumo-stm32/build /home/hector/Documentos/Robotica/firmware/minisumo-stm32/build/CMakeFiles/minisumo-stm32.hex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/minisumo-stm32.hex.dir/depend

