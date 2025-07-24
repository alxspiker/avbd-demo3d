/*
* spring.cpp - 3D AVBD Physics Engine
*
* Implements the 3D spring/distance constraint logic.
*/

#include "spring.h" // As before, good practice for self-containment
#include <float.h>

Spring::Spring(Solver* solver, Rigid* bodyA, Rigid* bodyB,
               const vec3& localAnchorA, const vec3& localAnchorB,
               float stiffness, float rest)
    : Force(solver, bodyA, bodyB), rA(localAnchorA), rB(localAnchorB), restLength(rest), springStiffness(stiffness)
{
    // Set the stiffness for the single constraint row this force manages.
    this->stiffness[0] = springStiffness;
    
    // If rest length is negative, use the initial distance as the rest length.
    if (this->restLength < 0) {
        vec3 pA = bodyA->position + rotate(bodyA->orientation, rA);
        vec3 pB = bodyB->position + rotate(bodyB->orientation, rB);
        this->restLength = length(pA - pB);
    }
    
    // Initialize other force parameters
    lambda[0] = 0.0f;
    penalty[0] = PENALTY_MIN;
    fmin[0] = -FLT_MAX;
    fmax[0] = FLT_MAX;
}


void Spring::computeConstraint(float dt) {
    // --- Calculate world-space anchor points ---
    quat qA = bodyA ? bodyA->orientation : quat();
    vec3 pA = bodyA ? bodyA->position + rotate(qA, rA) : rA;
    vec3 pB = bodyB->position + rotate(bodyB->orientation, rB);
    
    vec3 delta = pA - pB;
    float currentLength = length(delta);

    // --- Update Constraint Violation (C) ---
    // The violation is the difference between the current length and the rest length.
    C[0] = currentLength - restLength;

    // --- Pre-compute Hessian (H) ---
    // The Hessian of a distance constraint is H = (I - n ⊗ n) / length
    // This will be used by computeDerivatives. It's computed here because
    // the delta/normal is needed for C anyway.
    if (currentLength > VEC_EPSILON) {
        vec3 n = delta / currentLength;
        H_ll = (mat3() - outer_product(n, n)) / currentLength;
    } else {
        H_ll = mat3({0,0,0},{0,0,0},{0,0,0});
    }
}


void Spring::computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const {
    // --- Calculate World-Space Anchor Points and Delta Vector ---
    quat qA = bodyA ? bodyA->orientation : quat();
    vec3 pA = bodyA ? bodyA->position + rotate(qA, rA) : rA;
    vec3 pB = bodyB->position + rotate(bodyB->orientation, rB);
    vec3 delta = pA - pB;

    float currentLength = length(delta);
    if (currentLength < VEC_EPSILON) {
        J_linear = {0,0,0};
        J_angular = {0,0,0};
        return;
    }
    vec3 n = delta / currentLength;
    float sign = (body == bodyA) ? 1.0f : -1.0f;

    // --- Calculate Jacobian (J) ---
    // J_linear is simply the normalized direction vector.
    J_linear = n * sign;

    // J_angular is the cross product of the lever arm and the direction vector.
    if (body) {
        vec3 r_world = (body == bodyA) ? rotate(body->orientation, rA) : rotate(body->orientation, rB);
        J_angular = cross(r_world, n) * sign;
    } else { // bodyA is the world anchor
        J_angular = {0,0,0};
    }
    
    // NOTE: The Hessian (second derivative) was pre-computed in computeConstraint().
    // A full implementation would pass this back to the solver to add a geometric
    // stiffness term, improving stability. Our current 3D solver does not yet use this.
}

void Spring::draw() const {
    // Draw a line between the two anchor points.
    vec3 pA = bodyA->position + rotate(bodyA->orientation, rA);
    vec3 pB = bodyB->position + rotate(bodyB->orientation, rB);

    glDisable(GL_LIGHTING);
    glLineWidth(1.0f);
    
    // Draw the spring in a blue color
    glColor3f(0.2f, 0.2f, 0.9f);

    glBegin(GL_LINES);
    glVertex3fv(&pA.x);
    glVertex3fv(&pB.x);
    glEnd();
    
    glEnable(GL_LIGHTING);
}