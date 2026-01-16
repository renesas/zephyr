cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(zephyr_face_detection C CXX)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ai_application
    ${CMAKE_CURRENT_SOURCE_DIR}/ai_application/common
    ${CMAKE_CURRENT_SOURCE_DIR}/ai_application/face_detection
    ${CMAKE_CURRENT_SOURCE_DIR}/ai_application/face_detection/model
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

FILE(GLOB app_sources src/*.c src/*.cc)
FILE(GLOB ai_common ai_application/common/*.c ai_application/common/*.cc)
FILE(GLOB ai_face_detection ai_application/face_detection/*.c ai_application/face_detection/*.cc)

target_sources(app PRIVATE
    ${app_sources}
    ${ai_common}
    ${ai_face_detection}
)
