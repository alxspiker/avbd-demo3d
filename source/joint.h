/*
* joint.h - 3D AVBD Physics Engine
*
* Defines a 3D "weld" joint, which constrains the linear and angular
* motion between two bodies.
*/

#pragma once

#include "solver.h"

// A 3D fixed joint between two rigid bodies (or one body and the world).
// This constraint removes all 6 degrees of freedom.
struct Joint : Force {
    // Local-space anchor points on each body
    vec3 rA, rB;
    
    // The "rest" or initial relative orientation between the two bodies.
    quat initialRelativeOrientation;

    // We can use a single stiffness and motor value for all angular constraints.
    float angularStiffness;
    float angularMotor;
    float angularFracture;

    // Constructor for a joint between two bodies.
    Joint(Solver* solver, Rigid* bodyA, Rigid* bodyB,
          const vec3& localAnchorA, const vec3& localAnchorB,
          float linearStiffness = FLT_MAX, float angularStiffness = FLT_MAX,
          float motor = 0.0f, float fracture = FLT_MAX);
          
    // Constructor for a joint between a body and a world-space point.
    Joint(Solver* solver, Rigid* bodyB,
          const vec3& worldAnchor,
          float linearStiffness = FLT_MAX, float angularStiffness = FLT_MAX,
          float motor = 0.0f, float fracture = FLT_MAX);


    // --- Overridden virtual functions from the Force base class ---

    // This joint creates 6 constraint rows: 3 linear, 3 angular.
    int getRowCount() const override { return 6; }

    bool initialize() override;
    void computeConstraint(float dt) override;
    void computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const override;
    void draw() const override;
};