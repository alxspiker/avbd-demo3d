/*
* manifold.cpp - 3D AVBD Physics Engine
*
* CORRECTED: Implements full warm-starting with feature matching,
* static friction, and pre-computation for stabilization.
* Drawing code is now complete.
* UPDATED: Simplified constraint violation to reduce bounciness.
* UPDATED: Set stiffness to FLT_MAX for hard constraints to prevent falling through.
* UPDATED: Removed isManifold() definition (moved to solver.h).
* UPDATED: Flip C[0] sign to make C positive for penetration, reducing bounce.
* UPDATED: Added debug print for C[0] in computeConstraint.
* UPDATED: Changed C[0] to -separation - margin
* UPDATED: Added Baumgarte stabilization
* UPDATED: Flipped vrel to vA - vB
* UPDATED: Updated debug print with separation
* UPDATED: Moved vrel_now calculation before normal constraint to fix compile error
* UPDATED: Changed sign in computeDerivatives to (body == bodyA) ? -1.0f : 1.0f to push in correct direction.
* UPDATED: Changed C[0] to max(0.0f, separation - COLLISION_MARGIN) where separation positive for penetration.
* UPDATED: Removed Baumgarte velocity term to prevent divergence and unit mismatch.
* UPDATED: Flipped normal to point from B to A, and adjusted separation to dot(pB - pA, normal) for positive penetration.
* CORRECTED: Disabled friction in the position-based solver to prevent energy gain and sliding.
* CORRECTED: Added a small penetration slop to prevent jitter from solver over-correction.
*/

#include "solver.h"
#include <float.h>
#include <stdio.h> // For printf

// A small penetration tolerance to prevent jitter from solver over-correction.
const float PENETRATION_SLOP = 0.001f; // 1mm

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB)
    : Force(solver, bodyA, bodyB), numContacts(0)
{
    // The constructor is minimal, all work is done in initialize
}

int Manifold::getRowCount() const {
    return numContacts * 3; // 1 Normal, 2 Tangent friction rows per contact
}

bool Manifold::initialize() {
    combinedFriction = sqrtf(bodyA->friction * bodyB->friction);

    // --- Warm Starting Cache ---
    Contact oldContacts[4];
    float oldLambda[12];
    int oldNumContacts = numContacts;
    for (int i = 0; i < oldNumContacts; ++i) oldContacts[i] = contacts[i];
    for (int i = 0; i < oldNumContacts * 3; ++i) oldLambda[i] = lambda[i];

    // --- Get New Contacts ---
    numContacts = Manifold::collide(bodyA, bodyB, contacts, false);
    if (numContacts == 0) {
        return false;
    }

    // Set stiffness to FLT_MAX for hard constraints (normal and friction)
    for (int i = 0; i < getRowCount(); ++i) {
        stiffness[i] = FLT_MAX;
    }

    // --- Match New Contacts with Old for Warm Starting ---
    for (int i = 0; i < numContacts; ++i) {
        lambda[i*3 + 0] = lambda[i*3 + 1] = lambda[i*3 + 2] = 0.0f;
        penalty[i*3 + 0] = penalty[i*3 + 1] = penalty[i*3 + 2] = PENALTY_MIN;
        contacts[i].stick = false;

        for (int j = 0; j < oldNumContacts; ++j) {
            if (contacts[i].feature.value == oldContacts[j].feature.value) {
                lambda[i*3 + 0] = oldLambda[j*3 + 0];
                lambda[i*3 + 1] = oldLambda[j*3 + 1];
                lambda[i*3 + 2] = oldLambda[j*3 + 2];
                contacts[i].stick = oldContacts[j].stick;
                
                // For sticking contacts, reuse the exact previous contact points
                if(contacts[i].stick) {
                    contacts[i].rA = oldContacts[j].rA;
                    contacts[i].rB = oldContacts[j].rB;
                }
                break;
            }
        }
    }

    // --- Pre-computation for Stabilization ---
    for (int i = 0; i < numContacts; ++i) {
        vec3 world_rA = rotate(bodyA->orientation, contacts[i].rA);
        vec3 world_rB = rotate(bodyB->orientation, contacts[i].rB);

        vec3 vrel = (bodyA->linearVelocity + cross(bodyA->angularVelocity, world_rA)) -
                    (bodyB->linearVelocity + cross(bodyB->angularVelocity, world_rB));

        vec3 normal = contacts[i].normal;
        vec3 tangent1, tangent2;
        if (abs(normal.x) > 0.9f) tangent1 = normalize(cross(normal, vec3(0, 1, 0)));
        else tangent1 = normalize(cross(normal, vec3(1, 0, 0)));
        tangent2 = normalize(cross(normal, tangent1));

        contacts[i].C0_t.x = dot(vrel, tangent1);
        contacts[i].C0_t.y = dot(vrel, tangent2);
        contacts[i].C0_t.z = 0;
    }
    
    return true;
}

void Manifold::computeConstraint(float alpha) {
    for (int i = 0; i < numContacts; ++i) {
        // --- Normal Constraint (Penetration) ---
        vec3 world_rA = rotate(bodyA->orientation, contacts[i].rA);
        vec3 world_rB = rotate(bodyB->orientation, contacts[i].rB);
        vec3 pA = bodyA->position + world_rA;
        vec3 pB = bodyB->position + world_rB;

        vec3 normal = contacts[i].normal;
        
        // --- FIX ---
        // Use stored penetration depth for robust constraint formulation
        // The constraint represents the signed distance: negative = violation, positive/zero = satisfied
        // Since penetration depth is stored as positive when overlapping, we negate it for the constraint
        float penetration_depth = contacts[i].penetration;
        
        // Calculate current separation for verification, but use stored penetration as primary
        float current_separation = dot(pB - pA, normal); 
        
        // Use the more conservative (worse) constraint value between stored and current
        float constraint_from_stored = -penetration_depth + PENETRATION_SLOP;
        float constraint_from_current = current_separation - PENETRATION_SLOP;
        
        // Take the minimum (most violated) constraint
        C[i*3 + 0] = min(0.0f, min(constraint_from_stored, constraint_from_current));
        
        // Debug output for first few frames to verify constraint calculation
        static int debug_count = 0;
        if (debug_count < 20) {
            printf("Contact %d: penetration=%.4f, current_sep=%.4f, constraint=%.4f\n", 
                   i, penetration_depth, current_separation, C[i*3 + 0]);
            debug_count++;
        }

        // --- FIX ---
        // The position-based solver was incorrectly trying to resolve friction (a velocity
        // phenomenon) by applying positional corrections. This mismatch of units created a
        // feedback loop that injected energy, causing both the accelerating slide and the
        // bouncing from the post-stabilization step.
        //
        // By setting the tangential constraint violations to zero, we disable friction
        // in the position-based loop. All friction is now correctly and stably handled
        // by the dedicated velocity-impulse solver at the end of the Solver::step() function.
        C[i*3 + 1] = 0.0f;
        C[i*3 + 2] = 0.0f;
        
        // --- Update Force Limits for Friction Cone (No changes needed here) ---
        float friction_limit = combinedFriction * abs(lambda[i*3 + 0]);
        fmin[i*3 + 1] = -friction_limit;
        fmax[i*3 + 1] =  friction_limit;
        fmin[i*3 + 2] = -friction_limit;
        fmax[i*3 + 2] =  friction_limit;
        fmin[i*3 + 0] = 0;
        fmax[i*3 + 0] = FLT_MAX;
        
        // --- Sticking Logic (No changes needed here) ---
        float tangent_lambda = sqrtf(lambda[i*3+1]*lambda[i*3+1] + lambda[i*3+2]*lambda[i*3+2]);
        contacts[i].stick = tangent_lambda < friction_limit && length(contacts[i].C0_t) < STICK_THRESH;
    }
}


void Manifold::computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const {
    int contact_idx = row / 3;
    int constraint_type = row % 3;

    const Contact& c = contacts[contact_idx];
    vec3 world_r = (body == bodyA) ? rotate(body->orientation, c.rA) : rotate(body->orientation, c.rB);

    vec3 normal = c.normal;
    vec3 tangent1, tangent2;
    if (abs(normal.x) > 0.9f) tangent1 = normalize(cross(normal, vec3(0, 1, 0)));
    else tangent1 = normalize(cross(normal, vec3(1, 0, 0)));
    tangent2 = normalize(cross(normal, tangent1));
    
    vec3 basis;
    if (constraint_type == 0) basis = normal;
    else if (constraint_type == 1) basis = tangent1;
    else basis = tangent2;
    
    float sign = (body == bodyA) ? -1.0f : 1.0f; // Push bodyA up, bodyB down
    
    J_linear = basis * sign;
    J_angular = cross(world_r, basis) * sign;
}

void Manifold::draw() const
{
    if (!SHOW_CONTACTS) return;

    glDisable(GL_LIGHTING);
    glPointSize(6.0f);
    glLineWidth(2.0f);

    for (int i = 0; i < numContacts; ++i) {
        // Calculate the world-space contact points from both bodies' perspectives
        vec3 pA = bodyA->position + rotate(bodyA->orientation, contacts[i].rA);
        vec3 pB = bodyB->position + rotate(bodyB->orientation, contacts[i].rB);
        
        // The midpoint is a good representation of the contact location
        vec3 p_mid = (pA + pB) * 0.5f;

        // Draw the contact point itself. Yellow for sticking, purple for sliding.
        if(contacts[i].stick) {
            glColor3f(1.0f, 1.0f, 0.0f); // Yellow = Sticking
        } else {
            glColor3f(0.8f, 0.2f, 0.8f); // Purple = Sliding
        }
        
        glBegin(GL_POINTS);
        glVertex3fv(&p_mid.x);
        glEnd();
        
        // Draw the contact normal in red to show the direction of repulsive force.
        glColor3f(1.0f, 0.2f, 0.2f);
        glBegin(GL_LINES);
        glVertex3fv(&p_mid.x);
        vec3 end = p_mid + contacts[i].normal * 0.5f; // Draw normal with length 0.5 for visibility
        glVertex3fv(&end.x);
        glEnd();
    }

    glEnable(GL_LIGHTING);
}