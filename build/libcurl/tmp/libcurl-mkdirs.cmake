# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/email/Documents/Work/build/third-party/libcurl"
  "C:/Users/email/Documents/Work/build/libcurl/src/libcurl-build"
  "C:/Users/email/Documents/Work/build/libcurl"
  "C:/Users/email/Documents/Work/build/libcurl/tmp"
  "C:/Users/email/Documents/Work/build/libcurl/src/libcurl-stamp"
  "C:/Users/email/Documents/Work/build/libcurl/src"
  "C:/Users/email/Documents/Work/build/libcurl/src/libcurl-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/email/Documents/Work/build/libcurl/src/libcurl-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/email/Documents/Work/build/libcurl/src/libcurl-stamp${cfgdir}") # cfgdir has leading slash
endif()
