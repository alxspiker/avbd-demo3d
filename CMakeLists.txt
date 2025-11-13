cmake_minimum_required(VERSION 3.13)

# Project Name
project(avbd_demo3d)

# Add platform-specific definitions for Windows to prevent header conflicts
if(WIN32)
    add_definitions(-DWIN32_LEAN_AND_MEAN -DNOMINMAX)
endif()

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Automatically collect all source files from the "source" directory
file(GLOB_RECURSE SOURCES "source/*.cpp" "source/*.h")

# Add Executable
add_executable(${PROJECT_NAME} ${SOURCES})

if(WIN32)
    target_compile_definitions(avbd_demo3d PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

find_package(SDL2 REQUIRED)

# Add ImGui as a separate project
set(IMGUI_PROJECT_NAME "imgui")

set(IMGUI_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui")
if(NOT EXISTS "${IMGUI_SOURCE_DIR}/imgui.cpp")
    message(STATUS "Fetching Dear ImGui because external/imgui is empty")
    FetchContent_Declare(dear_imgui
        URL https://github.com/ocornut/imgui/archive/refs/tags/v1.90.4.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(dear_imgui)
    set(IMGUI_SOURCE_DIR "${dear_imgui_SOURCE_DIR}")
endif()

file(GLOB IMGUI_CORE
    "${IMGUI_SOURCE_DIR}/imgui.cpp"
    "${IMGUI_SOURCE_DIR}/imgui_draw.cpp"
    "${IMGUI_SOURCE_DIR}/imgui_tables.cpp"
    "${IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
)

set(IMGUI_BACKENDS
    "${IMGUI_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp"
    "${IMGUI_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
)

add_library(${IMGUI_PROJECT_NAME} STATIC ${IMGUI_CORE} ${IMGUI_BACKENDS})

set(SDL2_TARGET SDL2::SDL2)
if(TARGET SDL2::SDL2main)
    list(APPEND SDL2_TARGET SDL2::SDL2main)
endif()

target_include_directories(${IMGUI_PROJECT_NAME} PUBLIC
    "${IMGUI_SOURCE_DIR}"
    "${IMGUI_SOURCE_DIR}/backends"
)
if(DEFINED SDL2_INCLUDE_DIRS)
    target_include_directories(${IMGUI_PROJECT_NAME} PUBLIC ${SDL2_INCLUDE_DIRS})
endif()
target_link_libraries(${IMGUI_PROJECT_NAME} PUBLIC ${SDL2_TARGET})

target_link_libraries(${PROJECT_NAME} PRIVATE ${SDL2_TARGET} ${IMGUI_PROJECT_NAME})
if(DEFINED SDL2_INCLUDE_DIRS)
    target_include_directories(${PROJECT_NAME} PRIVATE ${SDL2_INCLUDE_DIRS})
endif()
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/source")

if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s LEGACY_GL_EMULATION=1 --shell-file ${CMAKE_SOURCE_DIR}/source/shell.html")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENGL_LIBRARIES})
    if(UNIX AND NOT APPLE)
        find_package(X11 REQUIRED)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${X11_LIBRARIES})
    endif()
endif()
