/*
* test_physics.cpp - Physics Validation Tests
* 
* This creates automated tests to validate realistic physics behavior
* without requiring the GUI. Tests measure actual physics properties.
*/

#include "source/solver.h"
#include <iostream>
#include <cmath>
#include <vector>

// Override draw functions to avoid OpenGL dependency
void Rigid::draw() const {}
void Manifold::draw() const {}

struct PhysicsTest {
    std::string name;
    bool passed;
    std::string details;
};

std::vector<PhysicsTest> runPhysicsTests() {
    std::vector<PhysicsTest> results;
    
    // Test 1: Mass affects acceleration (F = ma)
    {
        Solver solver;
        solver.defaultParams();
        
        // Create two objects with different masses, same size, dropped from same height
        Rigid* heavy = new Rigid(&solver, {1,1,1}, 5.0f, 0.5f, {0, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.1f); // 5 kg
        Rigid* light = new Rigid(&solver, {1,1,1}, 1.0f, 0.5f, {2, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.1f); // 1 kg
        
        // Run for 1 second (60 steps)
        for (int i = 0; i < 60; ++i) {
            solver.step();
        }
        
        // Both should have similar velocities (gravity affects all objects equally)
        float heavy_vel = length(heavy->linearVelocity);
        float light_vel = length(light->linearVelocity);
        float vel_diff = abs(heavy_vel - light_vel);
        
        bool passed = vel_diff < 1.0f; // Allow small difference due to numerical errors
        results.push_back({
            "Mass Independence Test", 
            passed,
            "Heavy object velocity: " + std::to_string(heavy_vel) + 
            ", Light object velocity: " + std::to_string(light_vel) +
            ", Difference: " + std::to_string(vel_diff)
        });
    }
    
    // Test 2: Restitution affects bouncing
    {
        Solver solver;
        solver.defaultParams();
        
        // Ground
        new Rigid(&solver, {100, 1, 100}, 0.0f, 0.5f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.3f);
        
        // Two balls with different restitution
        Rigid* bouncy = new Rigid(&solver, {1,1,1}, 1.0f, 0.1f, {0, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.9f);
        Rigid* dead = new Rigid(&solver, {1,1,1}, 1.0f, 0.1f, {2, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.0f);
        
        // Record initial heights
        float initial_height = 5.0f;
        
        // Run simulation until both settle
        float bouncy_max_height = 0.0f;
        float dead_max_height = 0.0f;
        bool bouncy_hit_ground = false;
        bool dead_hit_ground = false;
        
        for (int i = 0; i < 300; ++i) { // 5 seconds
            solver.step();
            
            // Track maximum height after first ground impact
            if (bouncy->position.y < 1.0f) bouncy_hit_ground = true;
            if (dead->position.y < 1.0f) dead_hit_ground = true;
            
            if (bouncy_hit_ground && bouncy->position.y > bouncy_max_height) {
                bouncy_max_height = bouncy->position.y;
            }
            if (dead_hit_ground && dead->position.y > dead_max_height) {
                dead_max_height = dead->position.y;
            }
        }
        
        bool passed = bouncy_max_height > dead_max_height + 1.0f; // Bouncy should bounce significantly higher
        results.push_back({
            "Restitution Bounce Test",
            passed,
            "Bouncy ball max height: " + std::to_string(bouncy_max_height) +
            ", Dead ball max height: " + std::to_string(dead_max_height)
        });
    }
    
    // Test 3: Energy conservation (within reasonable bounds)
    {
        Solver solver;
        solver.defaultParams();
        
        // Ground
        new Rigid(&solver, {100, 1, 100}, 0.0f, 0.5f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.3f);
        
        // Single object
        Rigid* object = new Rigid(&solver, {1,1,1}, 1.0f, 0.3f, {0, 5, 0}, quat(), {0,0,0}, {0,0,0}, 0.2f);
        
        // Calculate initial energy
        float initial_pe = object->mass * 9.81f * object->position.y;
        float initial_ke = 0.5f * object->mass * dot(object->linearVelocity, object->linearVelocity);
        float initial_energy = initial_pe + initial_ke;
        
        // Run simulation
        for (int i = 0; i < 180; ++i) { // 3 seconds
            solver.step();
        }
        
        // Calculate final energy
        float final_pe = object->mass * 9.81f * object->position.y;
        float final_ke = 0.5f * object->mass * dot(object->linearVelocity, object->linearVelocity);
        float final_energy = final_pe + final_ke;
        
        float energy_loss = initial_energy - final_energy;
        float energy_loss_percent = (energy_loss / initial_energy) * 100.0f;
        
        // Energy should decrease due to friction/damping, but not excessively
        bool passed = energy_loss_percent > 0 && energy_loss_percent < 95.0f;
        results.push_back({
            "Energy Conservation Test",
            passed,
            "Initial energy: " + std::to_string(initial_energy) +
            ", Final energy: " + std::to_string(final_energy) +
            ", Loss: " + std::to_string(energy_loss_percent) + "%"
        });
    }
    
    // Test 4: Friction affects motion
    {
        Solver solver;
        solver.defaultParams();
        
        // Ground with friction
        new Rigid(&solver, {100, 1, 100}, 0.0f, 0.8f, {0, -0.5f, 0}, quat(), {0,0,0}, {0,0,0}, 0.1f);
        
        // Two objects with different friction, same initial velocity
        Rigid* high_friction = new Rigid(&solver, {1,1,1}, 1.0f, 0.9f, {0, 1, 0}, quat(), {5,0,0}, {0,0,0}, 0.1f);
        Rigid* low_friction = new Rigid(&solver, {1,1,1}, 1.0f, 0.1f, {0, 1, 2}, quat(), {5,0,0}, {0,0,0}, 0.1f);
        
        // Run simulation
        for (int i = 0; i < 120; ++i) { // 2 seconds
            solver.step();
        }
        
        // High friction object should travel less distance
        float high_friction_distance = abs(high_friction->position.x);
        float low_friction_distance = abs(low_friction->position.x);
        
        bool passed = low_friction_distance > high_friction_distance + 1.0f;
        results.push_back({
            "Friction Effect Test",
            passed,
            "High friction distance: " + std::to_string(high_friction_distance) +
            ", Low friction distance: " + std::to_string(low_friction_distance)
        });
    }
    
    return results;
}

int main() {
    std::cout << "=== AVBD 3D Physics Engine Validation Tests ===" << std::endl;
    std::cout << "Testing realistic physics behavior..." << std::endl << std::endl;
    
    auto results = runPhysicsTests();
    
    int passed = 0;
    int total = results.size();
    
    for (const auto& test : results) {
        std::cout << "[" << (test.passed ? "PASS" : "FAIL") << "] " << test.name << std::endl;
        std::cout << "  Details: " << test.details << std::endl << std::endl;
        if (test.passed) passed++;
    }
    
    std::cout << "=== Test Results ===" << std::endl;
    std::cout << "Passed: " << passed << "/" << total << " tests" << std::endl;
    
    if (passed == total) {
        std::cout << "✅ All physics tests passed! The engine demonstrates realistic physics behavior." << std::endl;
        return 0;
    } else {
        std::cout << "❌ Some physics tests failed. The engine needs improvement." << std::endl;
        return 1;
    }
}