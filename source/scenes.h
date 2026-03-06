/*
* scenes.h - 3D AVBD Physics Engine
*
* Declares all test scenes and provides the data structures for the UI.
* This is a direct logical translation of the 2D scenes into a 3D context.
*/

#pragma once

#include "solver.h"
#include <cmath> // For M_PI

// We need to forward-declare the Force types we plan to use.
// Their implementation will come in a later step.
struct Joint;
struct Spring;
struct IgnoreCollision;


// --- Scene Definitions ---
// Each function clears the solver and sets up a specific test case.

static void sceneEmpty(Solver* solver) {
    solver->clear();
}

static void sceneGround(Solver* solver) {
    solver->clear();
    // A large, flat, static box to act as the ground.
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0});
}

static void sceneStack(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    // A simple stack of 10 cubes.
    for (int i = 0; i < 10; ++i) {
        new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, i * 1.1f + 0.5f, 0}, quat(), {0,0,0}, {0,0,0});
    }
}

static void scenePyramid(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    const int PYRAMID_SIZE = 10;
    for (int y = 0; y < PYRAMID_SIZE; ++y) {
        for (int x = 0; x < PYRAMID_SIZE - y; ++x) {
            float x_pos = (x - (PYRAMID_SIZE - y - 1) * 0.5f) * 1.1f;
            float y_pos = y * 1.05f + 0.5f;
            new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {x_pos, y_pos, 0}, quat(), {0,0,0}, {0,0,0});
        }
    }
}

static void sceneWall(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    const int W = 8, H = 8;
    const vec3 brickSize = {1.0f, 0.5f, 0.5f};
    const float spacingX = 1.03f; // tiny gap to avoid immediate overconstraint spikes
    const float spacingY = 0.52f;
    const float baseY = brickSize.y * 0.5f;

    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            float x_offset = (i % 2 == 0) ? 0.0f : 0.5f * spacingX;
            float x = (j - (W - 1) * 0.5f) * spacingX + x_offset;
            float y = i * spacingY + baseY;
            new Rigid(solver, brickSize, 1.0f, 0.4f, {x, y, -5}, quat(), {0,0,0}, {0,0,0});
        }
    }
}

static void sceneTwoBlockDrop(Solver* solver) {
    solver->clear();
    sceneGround(solver);

    // Bottom block resting on ground.
    new Rigid(solver, {1.0f, 1.0f, 1.0f}, 1.0f, 0.5f, {0.0f, 0.5f, 0.0f}, quat(), {0,0,0}, {0,0,0});

    // Top block starts offset and tilted so it tips, lands, and should settle
    // without spurious re-bounce.
    quat tilt(vec3(0.0f, 0.0f, 1.0f), 0.45f);
    new Rigid(solver, {1.0f, 1.0f, 1.0f}, 1.0f, 0.5f, {0.18f, 2.2f, 0.0f}, tilt, {0,0,0}, {0.0f, 0.0f, 1.0f});
}

static void sceneStress1000(Solver* solver) {
    solver->clear();
    sceneGround(solver);

    // Stress scene tuning: more solver work and gentler penalty ramping for
    // massive simultaneous contact events.
    solver->iterations = 20;
    solver->beta = 30000.0f;
    solver->gamma = 0.995f;

    // 10 x 10 x 10 = 1000 dynamic blocks dropping from high altitude.
    const int NX = 10;
    const int NY = 10;
    const int NZ = 10;
    const vec3 size = {1.0f, 1.0f, 1.0f};
    const float spacingXZ = 1.15f;
    const float spacingY = 2.0f;
    const float startY = 20.0f;
    const float jitterXZ = 0.04f;
    const float jitterY = 0.25f;

    auto hashFloat01 = [](unsigned int x) {
        x ^= x >> 16;
        x *= 0x7feb352dU;
        x ^= x >> 15;
        x *= 0x846ca68bU;
        x ^= x >> 16;
        return (x & 0x00FFFFFFU) / 16777215.0f;
    };

    for (int y = 0; y < NY; ++y) {
        for (int z = 0; z < NZ; ++z) {
            for (int x = 0; x < NX; ++x) {
                unsigned int seed = (unsigned int)(x + NX * (z + NZ * y) + 1);
                float jx = (hashFloat01(seed * 9781U) * 2.0f - 1.0f) * jitterXZ;
                float jz = (hashFloat01(seed * 6271U) * 2.0f - 1.0f) * jitterXZ;
                float jy = hashFloat01(seed * 3343U) * jitterY;

                float px = (x - (NX - 1) * 0.5f) * spacingXZ + jx;
                float py = startY + y * spacingY + jy;
                float pz = (z - (NZ - 1) * 0.5f) * spacingXZ + jz;
                new Rigid(solver, size, 1.0f, 0.5f, {px, py, pz}, quat(), {0, 0, 0}, {0, 0, 0});
            }
        }
    }
}

// NOTE: The following scenes for Joints, Springs, etc., are placeholders.
// They will cause compile errors until we define those Force types.
// This is expected and part of the top-down design process.

static void sceneRod(Solver* solver) {
    solver->clear();
    Rigid* prev = nullptr;
    const int N = 15;
    for (int i = 0; i < N; ++i) {
        vec3 pos = {0, 10.0f - i * 1.0f, 0};
        // The first segment is static (invMass = 0) to act as an anchor.
        Rigid* curr = new Rigid(solver, {0.25f, 1, 0.25f}, (i == 0) ? 0.0f : 1.0f, 0.5f, pos, quat(), {0,0,0}, {0,0,0});
        if (prev) {
            // Placeholder: new Joint(...)
        }
        prev = curr;
    }
}

static void sceneSoftBody(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    const int W = 10, H = 10;
    Rigid* grid[W][H];
    
    // Create a grid of cubes
    for (int i = 0; i < W; ++i) {
        for (int j = 0; j < H; ++j) {
            grid[i][j] = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 1.0f, 0.3f, {i*0.6f - W*0.3f, j*0.6f + 2.0f, 0}, quat(), {0,0,0}, {0,0,0});
        }
    }
    
    // Connect them with joints (placeholders)
    float stiffness = 1000.0f;
    for (int i = 0; i < W; ++i) {
        for (int j = 0; j < H; ++j) {
            if (i > 0) { /* new Joint(solver, grid[i-1][j], grid[i][j], ...); */ }
            if (j > 0) { /* new Joint(solver, grid[i][j-1], grid[i][j], ...); */ }
            if (i > 0 && j > 0) {
                 // Ignore diagonal collisions for stability in a cloth-like simulation
                 // new IgnoreCollision(solver, grid[i-1][j-1], grid[i][j]);
                 // new IgnoreCollision(solver, grid[i-1][j], grid[i][j-1]);
            }
        }
    }
}


// --- Scene Management Arrays ---
// These are used by the UI in main.cpp.

// The function pointer array for creating scenes.
static void (*scenes[])(Solver*) = {
    sceneEmpty,
    sceneGround,
    sceneStack,
    scenePyramid,
    sceneWall,
    sceneTwoBlockDrop,
    sceneStress1000,
    sceneRod,
    sceneSoftBody
};

// The corresponding names for the UI dropdown.
static const char* sceneNames[] = {
    "Empty",
    "Ground",
    "Stack",
    "Pyramid",
    "Wall",
    "TwoBlockDrop",
    "Stress1000",
    "Rod (WIP)",
    "Soft Body (WIP)"
};

// The total number of scenes.
static const int sceneCount = sizeof(scenes) / sizeof(scenes[0]);
