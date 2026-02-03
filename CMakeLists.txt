cmake_minimum_required(VERSION 3.13.0)
project(jolt-physics)

set(CMAKE_CXX_STANDARD 17)
# The COMPONENTS part checks that OGRE was built the way we need it
# The CONFIG flag makes sure we get OGRE instead of OGRE-next
find_package(Jolt REQUIRED)
find_package(OGRE REQUIRED COMPONENTS Bites Paging Terrain CONFIG)
find_package(ZLIB)
find_package(SDL2)
find_package(assimp REQUIRED CONFIG)
find_package(OgreProcedural REQUIRED CONFIG)

add_library(physics STATIC physics.cpp)
target_link_libraries(physics PUBLIC OgreMain Jolt::Jolt)
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(physics PRIVATE JPH_PROFILE_ENABLED)

