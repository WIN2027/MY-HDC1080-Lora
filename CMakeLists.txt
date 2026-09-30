cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 节点角色：两块板共用同一份代码，靠这个宏区分业务。
#   NODE_ROLE_TX = 采集 HDC1080 并经 SX1280 发送（本地串口出 0x18 遥测）
#   NODE_ROLE_RX = SX1280 连续接收，串口上报 0x19（含 RSSI/SNR）
# 用法：cmake -DNODE_ROLE=NODE_ROLE_RX ...   （默认 TX）
# 注意：CMake 的 -D 设的是缓存变量，必须在这里真正接到编译宏上，
#       否则 App_Config.h 里"构建时覆盖"的说法不成立。
set(NODE_ROLE "NODE_ROLE_TX" CACHE STRING "节点角色: NODE_ROLE_TX 或 NODE_ROLE_RX")

# Set the project name
set(CMAKE_PROJECT_NAME 2.4G)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}

        DOC/Src/FreeRTOS_Task.c
        DOC/Src/Back.c
        DOC/Src/MYUSART.c
        DOC/Inc/MYUSART.h
        DOC/Inc/MYUSART_CFG.h
        DOC/Src/HDC1080.c
        DOC/Inc/HDC1080.h
        DOC/Src/SX1280.c
        DOC/Inc/SX1280.h
        DOC/Inc/App_Config.h

        # 外挂模块硬件适配层与工作流（本工程新增）
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        DOC/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        NODE_ROLE=${NODE_ROLE}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
