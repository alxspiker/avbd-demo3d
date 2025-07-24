cmake_minimum_required(VERSION 3.13)

# Project Name
project(avbd_demo3d)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Explicitly list all source files with their subdirectory path.
set(SOURCES
    source/main.cpp
    source/solver.cpp
    source/rigid.cpp
    source/force.cpp
    source/collision.cpp
    source/manifold.cpp
    source/joint.cpp
    source/spring.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

# --- Dependency Management (ImGui and SDL2) ---
add_subdirectory(external/SDL)
set(IMGUI_PROJECT_NAME "imgui")
file(GLOB IMGUI_SRC
    external/imgui/*.cpp
    external/imgui/backends/imgui_impl_sdl2.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)
add_library(${IMGUI_PROJECT_NAME} STATIC ${IMGUI_SRC})
target_include_directories(${IMGUI_PROJECT_NAME} PUBLIC
    external/imgui
    external/imgui/backends
    external/SDL/include
)
target_link_libraries(${IMGUI_PROJECT_NAME} PUBLIC SDL2::SDL2 SDL2::SDL2main)

# --- Main Project Linking ---
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/source
    ${CMAKE_CURRENT_SOURCE_DIR}/external/SDL/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${IMGUI_PROJECT_NAME})

# --- Platform-Specific Linking (Simplified and Robust) ---
if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s LEGACY_GL_EMULATION=1 --shell-file source/shell.html")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
else()
    # Desktop Build (Windows, Linux, macOS)
    # The most robust, old-school way to link legacy OpenGL.
    
    find_package(OpenGL REQUIRED)
    find_package(GLU REQUIRED)
    
    # On Linux, OpenGL depends on the X11 libraries. We must find and link them too.
    if (UNIX AND NOT APPLE)
      find_package(X11 REQUIRED)
    endif()

    # Explicitly link all the libraries we found.
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLU_LIBRARIES}
    )
    if(X11_FOUND)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${X11_LIBRARIES})
    endif()
endif()

# Copy SDL2.dll to the output folder on Windows
if(WIN32)
    add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
endif()