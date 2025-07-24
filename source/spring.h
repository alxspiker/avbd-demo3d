/*
* spring.h - 3D AVBD Physics Engine
*
* Defines a 3D distance constraint (spring) between two bodies.
*/

#pragma once

#include "solver.h"

// A 3D distance constraint that acts like a spring.
// It creates a single constraint row to maintain a rest length.
struct Spring : Force {
    // Local-space anchor points on each body
    vec3 rA, rB;
    
    // The target length of the spring
    float restLength;

    // The stiffness of the spring (used by the solver)
    float springStiffness;

    // The pre-computed Hessian matrix (second derivative of the constraint)
    // In 3D, this is a 6x6 matrix, but we only need the 3x3 linear-linear part.
    mat3 H_ll;

    // Constructor
    Spring(Solver* solver, Rigid* bodyA, Rigid* bodyB,
           const vec3& localAnchorA, const vec3& localAnchorB,
           float stiffness, float rest = -1.0f);

    // --- Overridden virtual functions from the Force base class ---
    int getRowCount() const override { return 1; }
    bool initialize() override { return true; }
    void computeConstraint(float dt) override;
    void computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const override;
    void draw() const override;
};