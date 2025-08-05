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
    // A large, flat, static box to act as the ground with low restitution
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.2f);
}

static void sceneStack(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    // A simple stack of 10 cubes with moderate restitution
    for (int i = 0; i < 10; ++i) {
        new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, i * 1.05f + 0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.3f);
    }
}

static void scenePyramid(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    const int PYRAMID_SIZE = 10;
    for (int y = 0; y < PYRAMID_SIZE; ++y) {
        for (int x = 0; x < PYRAMID_SIZE - y; ++x) {
            float x_pos = (x - (PYRAMID_SIZE - y - 1) * 0.5f) * 1.1f;
            float y_pos = y * 0.95f + 0.5f;
            new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {x_pos, y_pos, 0}, quat(), {0,0,0}, {0,0,0}, 0.4f);
        }
    }
}

static void sceneWall(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    const int W = 8, H = 8;
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            float x_offset = (i % 2 == 0) ? 0.0f : 0.5f;
            new Rigid(solver, {1, 0.5f, 0.5f}, 1.0f, 0.4f, {j - W/2.0f + x_offset, i * 0.5f + 0.25f, -5}, quat(), {0,0,0}, {0,0,0}, 0.3f);
        }
    }
}

static void scenePhysicsTest(Solver* solver) {
    solver->clear();
    
    // Ground with realistic friction
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.7f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.3f);
    
    // Test 1: Different mass objects - Heavy box (high density) vs Light box (low density)
    new Rigid(solver, {1, 1, 1}, 5.0f, 0.4f, {-5, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.2f); // Heavy box
    new Rigid(solver, {1, 1, 1}, 1.0f, 0.4f, {-3, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.2f); // Light box
    
    // Test 2: Bouncing balls with different restitution
    new Rigid(solver, {0.5f, 0.5f, 0.5f}, 1.0f, 0.1f, {0, 8, 0}, quat(), {0,0,0}, {0,0,0}, 0.9f); // Super bouncy
    new Rigid(solver, {0.5f, 0.5f, 0.5f}, 1.0f, 0.9f, {2, 8, 0}, quat(), {0,0,0}, {0,0,0}, 0.0f); // Dead ball
    
    // Test 3: Rolling objects with initial velocity
    new Rigid(solver, {0.8f, 0.8f, 0.8f}, 2.0f, 0.3f, {4, 3, 0}, quat(), {3,0,0}, {0,0,0}, 0.4f); // Rolling object
    
    // Test 4: Projectile test - high velocity collision
    new Rigid(solver, {0.3f, 0.3f, 0.3f}, 3.0f, 0.2f, {-10, 2, 0}, quat(), {8,0,0}, {0,0,0}, 0.7f); // Fast projectile
    
    // Test 5: Stack stability test with mixed masses
    for (int i = 0; i < 3; ++i) {
        float density = 1.0f + i * 0.5f; // Increasing density
        float restitution = 0.1f + i * 0.1f; // Increasing restitution
        new Rigid(solver, {0.8f, 0.8f, 0.8f}, density, 0.6f, {6, i * 0.85f + 0.4f, 0}, quat(), {0,0,0}, {0,0,0}, restitution);
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
        Rigid* curr = new Rigid(solver, {0.25f, 1, 0.25f}, (i == 0) ? 0.0f : 1.0f, 0.5f, pos, quat(), {0,0,0}, {0,0,0}, 0.1f);
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
            grid[i][j] = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 1.0f, 0.3f, {i*0.6f - W*0.3f, j*0.6f + 2.0f, 0}, quat(), {0,0,0}, {0,0,0}, 0.2f);
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
    scenePhysicsTest,
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
    "Physics Test",
    "Rod (WIP)",
    "Soft Body (WIP)"
};

// The total number of scenes.
static const int sceneCount = sizeof(scenes) / sizeof(scenes[0]);