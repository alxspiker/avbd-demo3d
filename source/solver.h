/*
* solver.h - 3D AVBD Physics Engine
*
* CORRECTED: Replaced includes with forward declarations to break circular dependencies.
* UPDATED: Reverted COLLISION_MARGIN to 0.02f to stabilize contact detection.
* UPDATED: Added isManifold() declaration to Manifold struct.
*/

#pragma once

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "maths.h"

// --- Configuration Constants ---
#define MAX_CONSTRAINT_ROWS 12
#define PENALTY_MIN 1000.0f
#define PENALTY_MAX 1000000000.0f
#define COLLISION_MARGIN 0.02f
#define STICK_THRESH 0.02f
#define SHOW_CONTACTS true

// --- Forward Declarations ---
struct Rigid;
struct Force;
struct Manifold;
struct Joint;
struct Spring;
struct IgnoreCollision;
struct Solver;

// Holds all the state for a single 3D rigid body.
struct Rigid {
    Solver* solver;
    Force* forces;
    Rigid* next;

    int id; // Unique identifier for each body
    static int next_id; // Static counter

    vec3 position;
    quat orientation;
    vec3 linearVelocity, angularVelocity;
    vec3 prevLinearVelocity, prevAngularVelocity;

    vec3 initialPosition;
    quat initialOrientation;
    vec3 inertialPosition;
    quat inertialOrientation;

    vec3 size;
    float mass, invMass;
    mat3 invInertiaTensor;
    float friction;
    float radius;

    Rigid(Solver* solver, const vec3& size, float density, float friction, const vec3& pos, const quat& orient = quat(), const vec3& linVel = vec3(), const vec3& angVel = vec3());
    ~Rigid();

    mat3 getInvInertiaTensorWorld() const;
    mat3 getInertiaTensorWorld() const;
    bool isConstrainedTo(Rigid* other) const;
    void draw() const;
};

// Generic interface for all constraints.
struct Force {
    Solver* solver;
    Rigid* bodyA;
    Rigid* bodyB;
    Force* nextA, *nextB, *next;

    float C[MAX_CONSTRAINT_ROWS];
    float fmin[MAX_CONSTRAINT_ROWS], fmax[MAX_CONSTRAINT_ROWS];
    float lambda[MAX_CONSTRAINT_ROWS];
    float penalty[MAX_CONSTRAINT_ROWS];
    float motor[MAX_CONSTRAINT_ROWS];
    float stiffness[MAX_CONSTRAINT_ROWS];
    float fracture[MAX_CONSTRAINT_ROWS];

    Force(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    virtual ~Force();

    virtual int getRowCount() const = 0;
    virtual bool initialize() = 0;
    virtual void computeConstraint(float alpha) = 0;
    virtual void computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const = 0;
    virtual void draw() const {}
    virtual bool isManifold() const { return false; }
};

// Collision manifold between two bodies.
struct Manifold : Force {
    union FeaturePair {
        struct { unsigned char in_A, out_A, in_B, out_B; } e;
        int value;
    };
    struct Contact {
        FeaturePair feature;
        vec3 rA, rB;
        vec3 normal;
        float penetration;
        float C0_n;  // Stores the initial normal constraint violation for the Taylor series approximation used in stabilization
        vec3 C0_t;
        bool stick;
        // Accumulated impulses for warmstarting friction and normal in the velocity solve
        float jt1, jt2; // tangential accumulated impulses
        float jn;       // normal accumulated impulse this step (not persisted across frames)
    };
    Contact contacts[4];
    int numContacts;
    float combinedFriction;
    Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    int getRowCount() const override;
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const override;
    void draw() const override;
    static int collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts, bool flip);
    bool isManifold() const override { return true; } // UPDATED: Added declaration
};

// The core solver class.
struct Solver {
    float dt;
    vec3 gravity;
    int iterations;
    float alpha, beta, gamma;
    bool postStabilize;
    Rigid* bodies;
    Force* forces;

    Solver();
    ~Solver();

    void clear();
    void defaultParams();
    void step();
    void draw();
};