/*
* main.cpp - 3D AVBD Physics Engine
*
* CORRECTED: Re-added the missing typedefs for u32, etc.
*/

#include <stdio.h>
#include <stdint.h> // Required for uint32_t, etc.

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <SDL2/SDL.h>
#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_opengl3.h>

#include "solver.h"
#include "scenes.h"

// CORRECTED: Re-added the missing typedefs from the original project
typedef int32_t i32;
typedef uint32_t u32;
typedef int32_t b32;

// Global App State
b32 Running = 1;
SDL_Window *Window;
SDL_GLContext Context;
u32 WindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE;

// Engine and Interaction Objects
Solver* solver = nullptr;
Joint* drag = nullptr;

// Camera
vec3 camPos = {0, 10, 25};
vec3 camTarget = {0, 5, 0};
float camSpeed = 0.1f;

// Box creation properties
vec3 boxSize = {1, 1, 1};
vec3 boxVelocity = {0, 0, 0};
float boxFriction = 0.5f;
float boxDensity = 1.0f;
int currScene = 0;

void ui() {
    ImGui::Begin("Controls");
    ImGui::Text("Orbit Cam: Middle Mouse");
    ImGui::Text("Zoom Cam: Mouse Wheel");
    ImGui::Text("Pan Cam: Shift + Middle Mouse");
    ImGui::Text("Make Box: Right Mouse");
    ImGui::Text("Drag Box: Left Mouse");
    ImGui::Separator();

    if (ImGui::BeginCombo("Scene", sceneNames[currScene])) {
        for (int i = 0; i < sceneCount; i++) {
            bool selected = (currScene == i);
            if (ImGui::Selectable(sceneNames[i], selected)) {
                currScene = i;
                scenes[currScene](solver);
            }
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    
    if (ImGui::Button("Reset")) scenes[currScene](solver);
    ImGui::SameLine();
    if (ImGui::Button("Default Params")) solver->defaultParams();
    ImGui::Separator();

    ImGui::SliderFloat("Box Friction", &boxFriction, 0.0f, 2.0f);
    ImGui::SliderFloat3("Box Size", &boxSize.x, 0.1f, 10.0f);
    ImGui::SliderFloat3("Box Velocity", &boxVelocity.x, -20.0f, 20.0f);
    ImGui::Separator();

    ImGui::SliderFloat("Gravity Y", &solver->gravity.y, -20.0f, 20.0f);
    ImGui::SliderFloat("Dt", &solver->dt, 0.001f, 0.1f, "%.4f");
    ImGui::SliderInt("Iterations", &solver->iterations, 1, 50);
    ImGui::SliderFloat("Alpha", &solver->alpha, 0.0f, 1.0f);
    ImGui::SliderFloat("Beta", &solver->beta, 0.0f, 1000000.0f, "%.f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Gamma", &solver->gamma, 0.0f, 1.0f);
    ImGui::Checkbox("Post Stabilize", &solver->postStabilize);

    ImGui::End();
}

void input() {
    auto& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;

    // Camera Controls
    if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        vec3 forward = normalize(camTarget - camPos);
        vec3 right = normalize(cross(forward, vec3(0, 1, 0)));
        vec3 up = normalize(cross(right, forward));
        if (ImGui::IsKeyDown(ImGuiKey_LeftShift)) {
            camPos -= right * io.MouseDelta.x * camSpeed * 0.5f;
            camPos += up * io.MouseDelta.y * camSpeed * 0.5f;
            camTarget -= right * io.MouseDelta.x * camSpeed * 0.5f;
            camTarget += up * io.MouseDelta.y * camSpeed * 0.5f;
        } else {
            float yaw_angle = -io.MouseDelta.x * 0.005f;
            float pitch_angle = -io.MouseDelta.y * 0.005f;
            vec3 dir = camPos - camTarget;
            dir = rotate(quat(vec3(0, 1, 0), yaw_angle), dir);
            right = normalize(cross(dir, vec3(0, 1, 0)));
            dir = rotate(quat(right, pitch_angle), dir);
            camPos = camTarget + dir;
        }
    }
    camPos = camTarget + (camPos - camTarget) * powf(1.1f, -io.MouseWheel);

    // Create Box
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        vec3 createPos = camTarget + normalize(camTarget - camPos) * -5.0f;
        new Rigid(solver, boxSize, boxDensity, boxFriction, createPos, quat(), boxVelocity);
    }
}

void mainLoop() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT) Running = 0;
        if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) Running = 0;
    }

    int w, h;
    SDL_GetWindowSize(Window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)w / (double)h, 0.1, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camPos.x, camPos.y, camPos.z, camTarget.x, camTarget.y, camTarget.z, 0, 1, 0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    float light_pos[] = {10, 20, 30, 0};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ui();
    input();

    solver->step();
    solver->draw();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(Window);
}

int main(int argc, char** argv) {
    bool headless = false;
    const char* requestedScene = nullptr;
    int steps = 300; // Default number of steps for headless mode
    bool use2DPreset = false;
    bool useClassic = false;
    const char* csvPath = nullptr;

    solver = new Solver();
    // Parse command line arguments (solver must exist before applying trace flags)
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--nogfx") == 0 || strcmp(argv[i], "--headless") == 0) {
            headless = true;
        } else if ((strcmp(argv[i], "--scene") == 0 || strcmp(argv[i], "-s") == 0) && i + 1 < argc) {
            requestedScene = argv[++i];
        } else if ((strcmp(argv[i], "--steps") == 0 || strcmp(argv[i], "-n") == 0) && i + 1 < argc) {
            steps = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--trace-frame") == 0 && i + 1 < argc) {
            int f = atoi(argv[++i]);
            if (solver) solver->debugSingleFrame = f;
        } else if (strcmp(argv[i], "--trace-range") == 0 && i + 2 < argc) {
            int a = atoi(argv[++i]);
            int b = atoi(argv[++i]);
            if (solver) { solver->debugRangeStart = a; solver->debugRangeEnd = b; }
        } else if (strcmp(argv[i], "--trace-all") == 0) {
            if (solver) solver->debugAllConstraints = true;
        } else if (strcmp(argv[i], "--2dparams") == 0) {
            use2DPreset = true; // apply after solver construction & before scenes
        } else if (strcmp(argv[i], "--classic") == 0) {
            useClassic = true;
        } else if ((strcmp(argv[i], "--csv") == 0) && i + 1 < argc) {
            csvPath = argv[++i];
        }
    }

    // solver already created prior to argument parsing

    int sceneIdx = currScene;
    if (requestedScene) {
        for (int i = 0; i < sceneCount; ++i) {
            if (strcmp(sceneNames[i], requestedScene) == 0) {
                sceneIdx = i;
                break;
            }
        }
    }

    scenes[sceneIdx](solver);

    if (use2DPreset) {
        solver->apply2DParamPreset();
        printf("Applied 2D parameter preset (iterations=10, beta=100000, alpha=0.99, gamma=0.99, postStabilize=true)\n");
    }
    if (useClassic) {
        solver->classicMode = true;
        printf("Classic solver mode enabled (heuristics disabled).\n");
    }
    if (csvPath) {
        solver->csvLogEnabled = true;
        solver->csvLogPath = csvPath;
        solver->csvLogFile = fopen(csvPath, "w");
        if (solver->csvLogFile) {
            // Updated header to include jitter diagnostics columns added in solver.cpp
            fprintf(solver->csvLogFile, "frame,maxPen,avgEffPen,avgRawPen,avgPenalty,manifolds,contacts,jitRMSVel,jitRMSVelDelta,jitRMSVert,jitCOM,jitRMSContactN,topVy,totalLinCorr,manifoldsDropped,manifoldsResurrected\n");
            fflush(solver->csvLogFile);
            printf("CSV diagnostics logging to %s\n", csvPath);
        } else {
            fprintf(stderr, "Failed to open CSV log file: %s\n", csvPath);
            solver->csvLogEnabled = false;
        }
    }

    if (headless) {
        // Headless mode: run physics and print body states each step
        printf("Running in headless mode: scene '%s', steps=%d\n", sceneNames[sceneIdx], steps);
        for (int step = 0; step < steps; ++step) {
            solver->step();
            
            // Count manifolds and contacts for debugging
            int manifolds = 0, contacts = 0;
            for (Force* f = solver->forces; f; f = f->next) {
                if (dynamic_cast<Manifold*>(f)) {
                    manifolds++;
                    contacts += ((Manifold*)f)->numContacts;
                }
            }
            
            printf("Step %d: %d manifolds, %d contacts\n", step, manifolds, contacts);
            for (Rigid* body = solver->bodies; body != nullptr; body = body->next) {
                printf("  Body %d: Pos(%.4f, %.4f, %.4f)  ", body->id, body->position.x, body->position.y, body->position.z);
                printf("Rot(%.4f, %.4f, %.4f, %.4f)  ", body->orientation.x, body->orientation.y, body->orientation.z, body->orientation.w);
                printf("LinVel(%.4f, %.4f, %.4f)  ", body->linearVelocity.x, body->linearVelocity.y, body->linearVelocity.z);
                printf("AngVel(%.4f, %.4f, %.4f)\n", body->angularVelocity.x, body->angularVelocity.y, body->angularVelocity.z);
            }
        }
        delete solver;
        return 0;
    }

    // Normal graphics mode
    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    Window = SDL_CreateWindow("AVBD 3D", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, WindowFlags);
    Context = SDL_GL_CreateContext(Window);
    SDL_GL_MakeCurrent(Window, Context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(Window, Context);
    ImGui_ImplOpenGL3_Init("#version 150");

    scenes[sceneIdx](solver);

    while (Running) {
        mainLoop();
    }

    delete solver;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(Context);
    SDL_DestroyWindow(Window);
    SDL_Quit();
    return 0;
}
