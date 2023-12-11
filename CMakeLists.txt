cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
project(assignment_implement-controller)



file(GLOB prj_files src/*.cpp src/*.cc src/*.c src/*.h src/*.hpp)
add_executable(${PROJECT_NAME} ${prj_files})
target_sources(${PROJECT_NAME} PRIVATE ${prj_files})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

