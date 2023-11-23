cmake_minimum_required(VERSION 3.22)
project(cs  CXX C)

set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/gps)
include_directories(${PROJECT_SOURCE_DIR}/python)
include_directories(usr/include/python3.10)


# 3.加载线程库
find_package (Threads)
find_package (Python COMPONENTS Interpreter Development)
message(STATUS "Python_VERSION: ${Python_INCLUDE_DIRS}")


SET(LIBRARIES

        libminmea.a
        Threads::Threads

)
## 外部库的头文件
set(ZLIB_INCLUDE ${PROJECT_SOURCE_DIR}/lib/include)
## 外部库的lib文件（静态编译）
set(ZLIB_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/lib)

include_directories(${ZLIB_INCLUDE}
        ${PROJECT_SOURCE_DIR}/include
        ${Python_INCLUDE_DIRS})
link_directories(${ZLIB_LIBRARIES})

# 设置项目输出
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/clion_build)

# 设置编译文件
add_executable(cs
        CS.cpp
        include/REG.h
        include/wit_c_sdk.c
        include/wit_c_sdk.h
        include/seriald.c
        include/seriald.h
        gps/transform.c
        gps/transform.h
        lib/include/minmea.c
        )

#将第三方库链接在一起
target_link_libraries (${PROJECT_NAME} ${LIBRARIES} )
target_link_libraries(cs ${Python_LIBRARIES})

