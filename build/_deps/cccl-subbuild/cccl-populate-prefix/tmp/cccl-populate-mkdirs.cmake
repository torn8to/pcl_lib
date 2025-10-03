# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/nathan/pcl_lib/build/_deps/cccl-src")
  file(MAKE_DIRECTORY "/home/nathan/pcl_lib/build/_deps/cccl-src")
endif()
file(MAKE_DIRECTORY
  "/home/nathan/pcl_lib/build/_deps/cccl-build"
  "/home/nathan/pcl_lib/build/_deps/cccl-subbuild/cccl-populate-prefix"
  "/home/nathan/pcl_lib/build/_deps/cccl-subbuild/cccl-populate-prefix/tmp"
  "/home/nathan/pcl_lib/build/_deps/cccl-subbuild/cccl-populate-prefix/src/cccl-populate-stamp"
  "/home/nathan/pcl_lib/build/_deps/cccl-subbuild/cccl-populate-prefix/src"
  "/home/nathan/pcl_lib/build/_deps/cccl-subbuild/cccl-populate-prefix/src/cccl-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/nathan/pcl_lib/build/_deps/cccl-subbuild/cccl-populate-prefix/src/cccl-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/nathan/pcl_lib/build/_deps/cccl-subbuild/cccl-populate-prefix/src/cccl-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
