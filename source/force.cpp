/*
* force.cpp - 3D AVBD Physics Engine
*
* Implements the base Force class constructor and destructor, which handle
* the management of the solver's and bodies' linked lists for constraints.
*/

#include "solver.h"
#include <float.h>

Force::Force(Solver* solver, Rigid* bodyA, Rigid* bodyB)
    : solver(solver), bodyA(bodyA), bodyB(bodyB), nextA(0), nextB(0), next(0)
{
    // Add to the main solver linked list
    next = solver->forces;
    solver->forces = this;

    // Add to bodyA's linked list
    if (bodyA) {
        nextA = bodyA->forces;
        bodyA->forces = this;
    }

    // Add to bodyB's linked list
    if (bodyB) {
        nextB = bodyB->forces;
        bodyB->forces = this;
    }

    // Initialize all constraint rows to default "inactive" values.
    for (int i = 0; i < MAX_CONSTRAINT_ROWS; ++i) {
        stiffness[i] = 0.0f; // Default to soft
        lambda[i] = 0.0f;
        penalty[i] = 0.0f;
        motor[i] = 0.0f;
        fmin[i] = -FLT_MAX;
        fmax[i] = FLT_MAX;
        C[i] = 0.0f;
    }
}

Force::~Force() {
    // Remove from the main solver linked list
    Force** p = &solver->forces;
    while (*p != this) {
        p = &(*p)->next;
    }
    *p = next;

    // Remove from bodyA's linked list
    if (bodyA) {
        p = &bodyA->forces;
        while (*p != this) {
            // This traversal logic is complex but correct for a dual-linked list
            p = ((*p)->bodyA == bodyA) ? &(*p)->nextA : &(*p)->nextB;
        }
        *p = ((*p)->bodyA == bodyA) ? nextA : nextB;
    }

    // Remove from bodyB's linked list
    if (bodyB) {
        p = &bodyB->forces;
        while (*p != this) {
            p = ((*p)->bodyA == bodyB) ? &(*p)->nextA : &(*p)->nextB;
        }
        *p = ((*p)->bodyA == bodyB) ? nextA : nextB;
    }
}