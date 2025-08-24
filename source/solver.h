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
#include <cstdio>

// --- Configuration Constants ---
#define MAX_CONSTRAINT_ROWS 24
#define PENALTY_MIN 50.0f   // Reduced: start with a gentler baseline stiffness
#define PENALTY_MAX 100000.0f // Reduced: cap stiffness to avoid extreme position corrections
#define COLLISION_MARGIN 0.02f
#define STICK_THRESH 0.02f
#define CONTACT_PERSISTENCE_DISTANCE 0.01f  // Added: contacts within this distance are considered persistent
#define SHOW_CONTACTS true
// Enable periodic diagnostics printing (constraint error & penalty stats)
#define ENABLE_SOLVER_DIAGNOSTICS 1

// Diagnostics print period in frames (only used when ENABLE_SOLVER_DIAGNOSTICS != 0)
#define SOLVER_DIAG_PERIOD 30

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
    // Velocities captured before positional solve to decouple velocity update from corrections
    vec3 preSolveLinearVelocity;
    vec3 preSolveAngularVelocity;

    // Accumulated correction magnitudes within a single solver step (frame)
    // Used to enforce a hard per-frame cap on positional/orientation changes
    float frameLinearCorrectionAccum = 0.0f;
    float frameAngularCorrectionAccum = 0.0f;

    vec3 initialPosition;
    quat initialOrientation;
    vec3 inertialPosition;
    quat inertialOrientation;

    vec3 size;
    float mass, invMass;
    mat3 invInertiaTensor;
    float friction;
    float radius;

    // Count of stable supporting contacts this frame (set during manifold heavy update)
    int groundedContacts;
    // Aggregate upward normal support load accumulated this frame (approx from normal lambda)
    float supportLoad;
    // Flag if body is part of bottom support layer for early-phase preconditioning
    bool isBottomSupport;

    Rigid(Solver* solver, const vec3& size, float density, float friction, const vec3& pos, const quat& orient = quat(), const vec3& linVel = vec3(), const vec3& angVel = vec3());
    ~Rigid();

    mat3 getInvInertiaTensorWorld() const;
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
    // New: lightweight constraint recompute already uses stored effectivePenetration.
    // For manifolds we separate heavy state update (growth damping, stability, spike detection)
    // from per-iteration Jacobian C evaluation.
    virtual void updateConstraintState(float alpha) {} // default no-op
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
    // Instrumentation: effective penetration actually fed to solver after clamping
    float effectivePenetration; // capped version used for C calculation
    bool penetrationClamped;    // true if raw penetration exceeded cap
    float prevEffectivePen;     // previous frame's effective penetration (for growth damping)
        float C0_n;  // Stores the initial normal constraint violation for the Taylor series approximation used in stabilization
        vec3 C0_t;
        bool stick;
    // Stability tracking for adaptive penalty relaxation & creep mitigation
    int stableFrames;        // consecutive frames near bias depth
    bool relaxPenalty;       // flag: request penalty softening this frame
    bool spikeThisFrame;     // detected large penetration growth spike
    float prevRawPen;        // previous raw penetration (for growth diagnostics)
    int spikeCooldown;       // frames remaining of spike mitigation
    // Persistence hysteresis: remember last shallow settled effective penetration baseline
    float depthMemory;       // stored target shallow depth for hysteresis reactivation
    int memoryAge;           // frames since memory updated
    };
    Contact contacts[8];
    int numContacts;
    int computeCallsThisFrame; // instrumentation: number of computeConstraint invocations this frame
    float combinedFriction;
    // Persistence / hysteresis tracking (manifold-level)
    int framesAlive = 0;                // total frames this manifold has existed consecutively
    int framesSincePenetration = 0;      // consecutive frames with no real penetration (raw <= 0)
    bool resurrectedThisFrame = false;   // true if resurrected speculatively (hysteresis) this frame
    Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    int getRowCount() const override;
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void updateConstraintState(float alpha) override; // heavy state mutation once per frame
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
    int frameIndex; // counts simulation steps for periodic diagnostics
    // Diagnostics counters (reset each frame or period)
    int diagAxisClampCount = 0;
    int diagSpikeProjCount = 0;
    // Instrumentation / debug tracing controls
    int debugSingleFrame = -1;      // trace only this frame if >=0
    int debugRangeStart = -1;       // inclusive start of trace range
    int debugRangeEnd = -1;         // inclusive end of trace range
    bool debugAllConstraints = false; // trace every frame
    // Aggregated instrumentation counters (per frame)
    long long diagConstraintComputeCalls = 0; // total lightweight computeConstraint invocations (per-iteration)
        long long diagConstraintHeavyUpdateCalls = 0; // number of heavy updateConstraintState invocations (should ~= manifolds once per frame)
    int diagManifoldCount = 0;                // number of active manifolds this frame
    int diagManifoldsDropped = 0;             // manifolds removed this frame (post-hysteresis)
    int diagManifoldsResurrected = 0;         // manifolds kept alive speculatively this frame
        int diagContactCount = 0; // total contacts across all manifolds (post pruning)
    // Jitter diagnostics accumulators (reset each frame)
    double diagJitterVelSqSum = 0.0;
    double diagJitterVelDeltaSqSum = 0.0;
    double diagJitterVertSqSum = 0.0;
    int    diagJitterBodyCount = 0;
    double diagJitterContactNormalVelSqSum = 0.0;
    int    diagJitterContactCount = 0;
    vec3   prevCOM = {0,0,0};
    bool   hasPrevCOM = false;
    // Resting contact damping (new): reduces small oscillatory normal relative velocities
    float  restingNormalDampFactor = 0.35f; // fraction of relative normal velocity to remove when below threshold
    float  restingNormalDampThreshold = 0.40f; // m/s threshold defining "resting" along normal
    int    diagRestingNormalDampApplications = 0; // count of damping operations this frame
    // Per-phase invocation counters
    long long diagCallsProjection = 0;
    long long diagCallsPrimal = 0;      // total primal (all bodies)
    long long diagCallsDual = 0;
    long long diagCallsDiagnostics = 0;
    // Current phase tag for tracing
    const char* debugPhase = nullptr;
    int debugInvokingBodyId = -1; // body id whose loop invoked computeConstraint (for primal)
    // fullUpdatePhase flag obsolete after de-dup refactor; heavy update isolated to updateConstraintState path

    // Feature toggles
    bool classicMode = false;           // When true, bypass advanced heuristic phases for a minimal solver step
    bool csvLogEnabled = false;         // When true, write periodic diagnostics to csvLogFile
    const char* csvLogPath = nullptr;   // Path provided via CLI
    FILE* csvLogFile = nullptr;         // Open file handle

    Solver();
    ~Solver();

    void clear();
    void defaultParams();
    // Applies a parameter preset that mirrors the original 2D demo defaults
    // (useful for comparing behaviour or simplifying tuning):
    //  iterations=10, beta=100000, alpha=0.99, gamma=0.99, postStabilize=true
    void apply2DParamPreset();
    void step();
    void draw();
};

// Tracing helper (defined in solver.cpp)
bool solver_should_trace(const Solver* s);