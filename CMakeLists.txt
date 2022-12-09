cmake_minimum_required(VERSION 3.23)
project(CHashTable_benchmark C)

set(CMAKE_C_STANDARD 11)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(CGLM_SHARED OFF) # cglm
set(CGLM_STATIC ON) # cglm
set(CGLM_USE_C99 ON) # cglm

add_subdirectory(CEssentials)
add_subdirectory(cglm)

add_executable(CHashTable_benchmark main.c)
target_link_libraries(CHashTable_benchmark CEssentials cglm)
