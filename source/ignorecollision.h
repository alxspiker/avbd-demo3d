/*
* ignorecollision.h - 3D AVBD Physics Engine
*
* Defines the IgnoreCollision marker class.
*/

#pragma once

#include "solver.h"

// This is a "marker" force. It has no physical effect. Its only purpose
// is to be found by the `isConstrainedTo` check in the broadphase, which
// prevents a Manifold from being created between the two bodies.
struct IgnoreCollision : Force {
    IgnoreCollision(Solver* solver, Rigid* bodyA, Rigid* bodyB)
        : Force(solver, bodyA, bodyB) {}

    // --- Overridden virtual functions ---
    int getRowCount() const override { return 0; }
    bool initialize() override { return true; } // Always active
    void computeConstraint(float dt) override { /* No-op */ }
    void computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const override { /* No-op */ }
};