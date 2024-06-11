cmake_minimum_required(VERSION 2.8)
project(demo)

add_compile_options(-fPIC)

# set(PROTOBUF_INCLUDE_DIR "/usr/local/include")

# set(Protobuf_PROTOC_EXECUTABLE "/usr/local/bin/protoc")

# include(FindProtobuf)
find_package(Protobuf REQUIRED)
    if(NOT PROTOBUF_FOUND)
        message(FATAL_ERROR "Protobuf not found")
    endif()
include_directories(
    ${PROTOBUF_INCLUDE_DIR}
    )

# added for image loading
find_package(OpenCV 4.7)
    if(NOT OpenCV_FOUND)
       message(FATAL_ERROR "OpenCV > 4.7 not found.")
    endif()

# file(GLOB PROTO_FILES ${PROJECT_SOURCE_DIR}/proto/*.proto)
file(GLOB PROTO_FILES ${PROJECT_SOURCE_DIR}/proto/offline_bbox.proto)
message("PROTO_FILES: ${PROTO_FILES}")
foreach(proto_name ${PROTO_FILES})
      file(RELATIVE_PATH proto_rel_name ${PROJECT_SOURCE_DIR} ${proto_name})
      list(APPEND REL_PROTO_FILES ${proto_rel_name})
endforeach()
message("REL_PROTO_FILES: ${REL_PROTO_FILES}")

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS  ${PROJECT_SOURCE_DIR}/${REL_PROTO_FILES})
message("PROTO_SRCS: ${PROTO_SRCS}")
message("PROTO_HDRS: ${PROTO_HDRS}")
add_library(proto_lib STATIC ${PROTO_SRCS} ${PROTO_HDRS})
message("PROTOBUF_INCLUDE_DIR: ${PROTOBUF_INCLUDE_DIR}")
target_include_directories(proto_lib PUBLIC ${PROTOBUF_INCLUDE_DIR})
target_link_libraries(proto_lib PRIVATE ${PROTOBUF_STATIC_LIBRARIES})

find_library(HIREDIS_LIB hiredis)
find_library(REDIS_PLUS_PLUS_LIB redis++)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} 
    ${PROTO_SRCS}
    ${PROTO_HDRS}
    )

target_link_libraries(
    ${PROJECT_NAME}
    ${PROTOBUF_LIBRARY}
    ${OpenCV_LIBS}  # added for image loading
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 单目脚本 
add_executable(subscribe_semanticMono
    ${PROJECT_SOURCE_DIR}/src/subscribe_semanticMono.cc
    )
target_link_libraries(subscribe_semanticMono ${PROJECT_NAME})
target_link_libraries(subscribe_semanticMono ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})

# 双目脚本
add_executable(subscribe_semanticStereo
    ${PROJECT_SOURCE_DIR}/src/subscribe_semanticStereo.cc
    )
target_link_libraries(subscribe_semanticStereo ${PROJECT_NAME})
target_link_libraries(subscribe_semanticStereo ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})

# 收发图并行通道-02
add_executable(subscribe_semanticStereo02
    ${PROJECT_SOURCE_DIR}/src/subscribe_semanticStereo02.cc
    )
target_link_libraries(subscribe_semanticStereo02 ${PROJECT_NAME})
target_link_libraries(subscribe_semanticStereo02 ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})

# 收发图并行通道-03
add_executable(subscribe_semanticStereo03
    ${PROJECT_SOURCE_DIR}/src/subscribe_semanticStereo03.cc
    )
target_link_libraries(subscribe_semanticStereo03 ${PROJECT_NAME})
target_link_libraries(subscribe_semanticStereo03 ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})

# 收发图并行通道-04
add_executable(subscribe_semanticStereo04
    ${PROJECT_SOURCE_DIR}/src/subscribe_semanticStereo04.cc
    )
target_link_libraries(subscribe_semanticStereo04 ${PROJECT_NAME})
target_link_libraries(subscribe_semanticStereo04 ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})

# add_executable(subscribe_matching
#     ${PROJECT_SOURCE_DIR}/src/subscribe_matching.cc
#     )
# target_link_libraries(subscribe_matching ${PROJECT_NAME})
# target_link_libraries(subscribe_matching ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})

# add_executable(publish_test
#     ${PROJECT_SOURCE_DIR}/src/publish_test.cc
#     )
# target_link_libraries(publish_test ${PROJECT_NAME})
# target_link_libraries(publish_test ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})

