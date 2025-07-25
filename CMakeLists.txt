cmake_minimum_required(VERSION 3.13)

# Project Name
project(avbd_demo3d)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add platform-specific definitions for Windows.
# This is best practice when dealing with Windows headers (which GLAD will include for you).
if(WIN32)
    add_compile_definitions(WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# Automatically collect all source files from the "source" directory
file(GLOB_RECURSE SOURCES "source/*.cpp" "source/*.h")

# Add Executable
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/source")

# --- Dependencies ---

# Include SDL2
add_subdirectory(external/SDL)

# Add ImGui as a static library
set(IMGUI_PROJECT_NAME "imgui")
file(GLOB IMGUI_SRC
    external/imgui/*.cpp
    external/imgui/backends/imgui_impl_sdl2.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)
add_library(${IMGUI_PROJECT_NAME} STATIC ${IMGUI_SRC})

# Configure ImGui includes and link it to SDL2
target_include_directories(${IMGUI_PROJECT_NAME} PUBLIC
    external/imgui
    external/imgui/backends
)
target_link_libraries(${IMGUI_PROJECT_NAME} PUBLIC SDL2::SDL2 SDL2::SDL2main)


# --- Platform-Specific Configuration & Linking ---

if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    # --- Emscripten Build ---
    set(SHELL_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/source/shell.html")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 --shell-file ${SHELL_FILE_PATH}")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    # Link main project to ImGui (which transitively links SDL2)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${IMGUI_PROJECT_NAME})

else()
    # --- Desktop Build (Windows, Linux, macOS) ---
    find_package(OpenGL REQUIRED)
    find_package(GLU REQUIRED)
    
    # NEW: Find the GLAD package
    find_package(glad REQUIRED)

    if (UNIX AND NOT APPLE)
      find_package(X11 REQUIRED)
    endif()

    # Link the main project to ImGui and all required graphics libraries
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        ${IMGUI_PROJECT_NAME}   # Links ImGui and transitively SDL2
        OpenGL::GL
        GLU::GLU
        glad::glad              # NEW: Link against the GLAD library
    )

    if(X11_FOUND)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${X11_LIBRARIES})
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
endif()
