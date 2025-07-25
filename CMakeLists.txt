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

# Automatically collect all source files from the "source" directory
file(GLOB_RECURSE SOURCES "source/*.cpp" "source/*.h")

# Add Executable
add_executable(${PROJECT_NAME} ${SOURCES})

if(WIN32)
    target_compile_definitions(avbd_demo3d PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# Include SDL2 first (ImGui needs it)
add_subdirectory(external/SDL)

# Add ImGui as a separate project
set(IMGUI_PROJECT_NAME "imgui")

file(GLOB IMGUI_SRC
    external/imgui/*.cpp
    external/imgui/backends/imgui_impl_sdl2.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)

add_library(${IMGUI_PROJECT_NAME} STATIC ${IMGUI_SRC})

# Set include directories for ImGui
target_include_directories(${IMGUI_PROJECT_NAME} PUBLIC
    external/imgui
    external/imgui/backends
    external/SDL/include
)

# Link ImGui with SDL2
target_link_libraries(${IMGUI_PROJECT_NAME} PUBLIC SDL2::SDL2 SDL2::SDL2main)

# Link the main project with SDL2 and ImGui
target_include_directories(${PROJECT_NAME} PRIVATE external/SDL/include)
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2 SDL2::SDL2main ${IMGUI_PROJECT_NAME})

if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s LEGACY_GL_EMULATION=1 --shell-file ../source/shell.html")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
else()
    # Find and Link OpenGL
    find_package(OpenGL REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)
endif()

# Copy SDL2.dll to the output folder on Windows
if(WIN32)
    add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>    # Path to the built SDL2.dll
            $<TARGET_FILE_DIR:${PROJECT_NAME}> # Output directory of the executable
    )
endif()

# --- Main Project Linking ---
target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/source"
)
# This is now sufficient, as linking to imgui will transitively link to SDL.
target_link_libraries(${PROJECT_NAME} PRIVATE ${IMGUI_PROJECT_NAME})

# --- Platform-Specific Linking ---
if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s LEGACY_GL_EMULATION=1 --shell-file ${CMAKE_SOURCE_DIR}/source/shell.html")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
else()
    # Desktop Build
    find_package(OpenGL REQUIRED)
    find_package(GLU REQUIRED)
    
    if (UNIX AND NOT APPLE)
      find_package(X11 REQUIRED)
    endif()

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
            # Use the underlying target name here as well
            $<TARGET_FILE:SDL2-static>
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
endif()
