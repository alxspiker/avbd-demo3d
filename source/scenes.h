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
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.3f);
}

static void sceneStack(Solver* solver) {
    solver->clear();
    sceneGround(solver);
    // A simple stack of 10 cubes.
    for (int i = 0; i < 10; ++i) {
        new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, i * 1.05f + 0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.2f);
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
            new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {x_pos, y_pos, 0}, quat(), {0,0,0}, {0,0,0}, 0.3f);
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
            new Rigid(solver, {1, 0.5f, 0.5f}, 1.0f, 0.4f, {j - W/2.0f + x_offset, i * 0.5f + 0.25f, -5}, quat(), {0,0,0}, {0,0,0}, 0.4f);
        }
    }
}

static void scenePhysicsTest(Solver* solver) {
    solver->clear();
    
    printf("=== AVBD 3D Comprehensive Physics Test ===\n");
    printf("Testing realistic physics behaviors with AVBD core\n\n");
    
    // Enable realistic physics parameters
    solver->setRealisticPhysics();
    
    // Ground with realistic friction
    Rigid* ground = new Rigid(solver, {100, 1, 100}, 0.0f, 0.7f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.3f);
    printf("Ground created: size=(100, 1, 100), friction=0.7, restitution=0.3, mass=infinite (static)\n");
    
    // Test 1: Heavy object (100 lbs = ~45.36 kg) - should bounce realistically
    // Using density to achieve proper mass: mass = volume * density
    // Volume of 1x1x1 box = 1 m³, so density = 45.36 kg/m³
    float heavy_density = 45.36f; // kg/m³ to get ~100 lbs
    Rigid* heavy_box = new Rigid(solver, {1, 1, 1}, heavy_density, 0.4f, {-5, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.2f);
    printf("Heavy box (100 lbs): mass=%.2f kg, friction=0.4, restitution=0.2 (low bounce)\n", heavy_box->mass);
    
    // Test 2: Light object (10 lbs = ~4.54 kg) - should bounce more
    float light_density = 4.54f;
    Rigid* light_box = new Rigid(solver, {0.8f, 0.8f, 0.8f}, light_density, 0.3f, {-3, 6, 0}, quat(), {0,0,0}, {0,0,0}, 0.6f);
    printf("Light box (10 lbs): mass=%.2f kg, friction=0.3, restitution=0.6 (medium bounce)\n", light_box->mass);
    
    // Test 3: Rolling objects - spherical-like behavior with different sizes
    Rigid* rolling_small = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 2.0f, 0.2f, {0, 3, 0}, quat(), {2,0,0}, {0,0,0}, 0.4f);
    printf("Small rolling object: mass=%.2f kg, initial velocity=(2,0,0), friction=0.2, restitution=0.4\n", rolling_small->mass);
    
    Rigid* rolling_large = new Rigid(solver, {1.5f, 1.5f, 1.5f}, 8.0f, 0.3f, {2, 4, 0}, quat(), {-1.5f,0,0}, {0,0,0}, 0.3f);
    printf("Large rolling object: mass=%.2f kg, initial velocity=(-1.5,0,0), friction=0.3, restitution=0.3\n", rolling_large->mass);
    
    // Test 4: Stacking test with different masses
    for (int i = 0; i < 5; ++i) {
        float stack_density = 1.0f + i * 0.5f; // Increasing density
        float stack_restitution = 0.1f + i * 0.1f; // Increasing restitution
        Rigid* stack_box = new Rigid(solver, {0.8f, 0.6f, 0.8f}, stack_density, 0.6f, 
                                   {5, i * 0.65f + 0.3f, 0}, quat(), {0,0,0}, {0,0,0}, stack_restitution);
        printf("Stack box %d: mass=%.2f kg, friction=0.6, restitution=%.1f\n", i+1, stack_box->mass, stack_restitution);
    }
    
    // Test 5: Bouncing balls with different restitution
    Rigid* bouncy_ball = new Rigid(solver, {0.6f, 0.6f, 0.6f}, 2.0f, 0.1f, {7, 8, 0}, quat(), {0,0,0}, {0,0,0}, 0.9f);
    printf("Bouncy ball: mass=%.2f kg, friction=0.1, restitution=0.9 (super bouncy)\n", bouncy_ball->mass);
    
    Rigid* dead_ball = new Rigid(solver, {0.6f, 0.6f, 0.6f}, 2.0f, 0.9f, {9, 8, 0}, quat(), {0,0,0}, {0,0,0}, 0.0f);
    printf("Dead ball: mass=%.2f kg, friction=0.9, restitution=0.0 (no bounce)\n", dead_ball->mass);
    
    // Test 6: Pendulum-like motion
    Rigid* pendulum = new Rigid(solver, {0.3f, 2.0f, 0.3f}, 3.0f, 0.4f, {-8, 6, 2}, 
                               quat({0,0,1}, 0.5f), {0,0,0}, {0,0,0}, 0.5f);
    printf("Pendulum rod: mass=%.2f kg, initial rotation=0.5 rad around Z-axis, restitution=0.5\n", pendulum->mass);
    
    // Test 7: High-energy collision test
    Rigid* projectile = new Rigid(solver, {0.4f, 0.4f, 0.4f}, 5.0f, 0.2f, {-10, 3, 0}, quat(), {8,0,0}, {0,0,0}, 0.8f);
    printf("Projectile: mass=%.2f kg, initial velocity=(8,0,0), restitution=0.8 (high energy collision)\n", projectile->mass);
    
    printf("\n=== Test Parameters ===\n");
    printf("Gravity: (%.1f, %.1f, %.1f) m/s²\n", solver->gravity.x, solver->gravity.y, solver->gravity.z);
    printf("Time step: %.4f s (%.0f Hz)\n", solver->dt, 1.0f/solver->dt);
    printf("Solver iterations: %d\n", solver->iterations);
    printf("Beta (penalty stiffness): %.0f\n", solver->beta);
    printf("Alpha (stabilization): %.3f\n", solver->alpha);
    printf("Gamma (decay): %.3f\n", solver->gamma);
    printf("\n=== Expected Behaviors ===\n");
    printf("- Heavy box should fall and bounce less than light box\n");
    printf("- Rolling objects should exhibit realistic rolling motion with friction\n");
    printf("- Stack should remain stable with proper weight distribution\n");
    printf("- Bouncy ball should bounce much more than dead ball\n");
    printf("- Pendulum should exhibit rotational dynamics\n");
    printf("- Projectile should create high-energy collisions\n");
    printf("- All objects should respect conservation of momentum and energy\n");
    printf("- Restitution should control bounce behavior realistically\n");
    printf("\n=== Physics Test Started ===\n\n");
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