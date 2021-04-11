cmake_minimum_required(VERSION 3.17)
project(SLAM)

set(CMAKE_CXX_STANDARD 11)
# Add header file include directories
include_directories(./include)
# Add block directories
#set(SRC src/workerManger.cpp  src/employee.cpp  src/manager.cpp src/boss.cpp)

# Target
#add_executable(CompanySystem main.cpp ${SRC} )

add_executable(SLAM src/main.cpp include/HgDataParser.h include/matops.h include/Serial.h include/filter_function.h src/HgDataParser.cpp src/matops.c src/Serial.cpp)