/*
 * solver.cpp - 3D AVBD Physics Engine
 *
 * Reworked to use a stable 6x6 block solve with full coupling, conservative
 * warm-starting, and robust diagnostics so the 3D translation of the AVBD
 * method matches the behaviour of the reference 2D solver.
 */

#include "solver.h"

#include <cmath>
#include <cfloat>
#include <cstdio>

namespace {

struct vec6 {
    vec3 l;
    vec3 a;
};

struct mat66 {
    mat3 ll;
    mat3 la;
    mat3 al;
    mat3 aa;
};

static mat3 zeroMatrix()
{
    return mat3(vec3(), vec3(), vec3());
}

static mat3 outer(const vec3& a, const vec3& b)
{
    return mat3(a * b.x, a * b.y, a * b.z);
}

static bool isFiniteVec3(const vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static bool isFiniteQuat(const quat& q)
{
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

static void sanitizeVec3(vec3& v, const char* label, int bodyId)
{
    if (!isFiniteVec3(v)) {
        std::printf("[Physics] Warning: body %d produced non-finite %s (%.3f, %.3f, %.3f); resetting to zero.\n",
                    bodyId, label, v.x, v.y, v.z);
        v = vec3();
    }
}

static void sanitizeQuat(quat& q, const char* label, int bodyId)
{
    if (!isFiniteQuat(q)) {
        std::printf("[Physics] Warning: body %d produced non-finite %s; resetting to identity.\n", bodyId, label);
        q = quat();
    }
}

static vec6 solve6x6(const mat66& A, const vec6& b)
{
    vec3 col0 = solve(A.ll, A.la.cols[0]);
    vec3 col1 = solve(A.ll, A.la.cols[1]);
    vec3 col2 = solve(A.ll, A.la.cols[2]);

    mat3 AinvB(col0, col1, col2);

    vec3 x0 = solve(A.ll, b.l);
    mat3 schur = A.aa - A.al * AinvB;
    vec3 rhs_s = b.a - A.al * x0;
    vec3 y = solve(schur, rhs_s);
    vec3 x = x0 - AinvB * y;

    return {x, y};
}

static void clampAngularVelocity(vec3& w)
{
    const float maxSpeed = 80.0f;
    float len = length(w);
    if (len > maxSpeed && len > VEC_EPSILON) {
        w *= maxSpeed / len;
    }
}

} // namespace

Solver::Solver()
    : bodies(nullptr)
    , forces(nullptr)
    , enableDiagnostics(false)
    , logFrequency(60)
    , stepIndex(0)
    , lastDiagnostics{}
{
    defaultParams();
}

Solver::~Solver()
{
    clear();
}

void Solver::clear()
{
    while (forces) delete forces;
    while (bodies) delete bodies;
    bodies = nullptr;
    forces = nullptr;
    stepIndex = 0;
    lastDiagnostics = Diagnostics{};
}

void Solver::defaultParams()
{
    dt = 1.0f / 60.0f;
    gravity = vec3(0.0f, -10.0f, 0.0f);
    iterations = 10;
    alpha = 0.9f;
    beta = 100000.0f;
    gamma = 0.99f;
    postStabilize = true;
    if (logFrequency <= 0) {
        logFrequency = 60;
    }
    lastDiagnostics = Diagnostics{};
}

void Solver::step()
{
    ++stepIndex;

    Diagnostics stats{};

    // --- 1. Broadphase ---
    for (Rigid* bodyA = bodies; bodyA != nullptr; bodyA = bodyA->next) {
        for (Rigid* bodyB = bodyA->next; bodyB != nullptr; bodyB = bodyB->next) {
            vec3 dp = bodyA->position - bodyB->position;
            float r = bodyA->radius + bodyB->radius;
            if (dot(dp, dp) <= r * r && !bodyA->isConstrainedTo(bodyB)) {
                new Manifold(this, bodyA, bodyB);
            }
        }
    }

    // --- 2. Initialize and Warmstart Forces ---
    for (Force* force = forces; force != nullptr; ) {
        if (!force->initialize()) {
            Force* next = force->next;
            delete force;
            force = next;
            continue;
        }

        int rows = force->getRowCount();
        for (int i = 0; i < rows; ++i) {
            if (postStabilize) {
                force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
            } else {
                force->lambda[i] *= alpha * gamma;
                force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
            }

            if (force->stiffness[i] > 0.0f && force->stiffness[i] < FLT_MAX) {
                force->penalty[i] = min(force->penalty[i], force->stiffness[i]);
            }
        }

        force = force->next;
    }

    // --- 3. Predict Body States ---
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        body->prevLinearVelocity = body->linearVelocity;
        body->prevAngularVelocity = body->angularVelocity;
        clampAngularVelocity(body->angularVelocity);

        body->initialPosition = body->position;
        body->initialOrientation = body->orientation;

        body->inertialPosition = body->position;
        body->inertialOrientation = body->orientation;

        if (body->invMass > 0.0f) {
            ++stats.dynamicBodies;

            sanitizeVec3(body->linearVelocity, "linear velocity", body->id);
            sanitizeVec3(body->angularVelocity, "angular velocity", body->id);

            body->inertialPosition = body->position + body->linearVelocity * dt + gravity * (dt * dt);

            quat omega(body->angularVelocity.x, body->angularVelocity.y, body->angularVelocity.z, 0.0f);
            body->inertialOrientation = normalize(body->orientation + (omega * body->orientation) * (0.5f * dt));

            float gravityLen = length(gravity);
            float accelWeight = 0.0f;
            if (gravityLen > 1e-5f) {
                vec3 accel = (body->linearVelocity - body->prevLinearVelocity) / dt;
                float projected = dot(accel, gravity / gravityLen);
                accelWeight = clamp(projected / gravityLen, 0.0f, 1.0f);
                if (!std::isfinite(accelWeight)) accelWeight = 0.0f;
            }

            body->position += body->linearVelocity * dt + gravity * (accelWeight * dt * dt);
            body->orientation = body->inertialOrientation;

            sanitizeVec3(body->position, "predicted position", body->id);
            sanitizeQuat(body->orientation, "predicted orientation", body->id);
        } else {
            body->position = body->inertialPosition;
            body->orientation = body->inertialOrientation;
        }
    }

    // --- 4. Main Iterative Solver Loop ---
    int totalIterations = iterations + (postStabilize ? 1 : 0);
    for (int it = 0; it < totalIterations; ++it) {
        float currentAlpha = postStabilize ? (it < iterations ? 1.0f : 0.0f) : alpha;

        for (Rigid* body = bodies; body != nullptr; body = body->next) {
            if (body->invMass <= 0.0f) {
                continue;
            }

            mat66 lhs;
            lhs.ll = zeroMatrix();
            lhs.la = zeroMatrix();
            lhs.al = zeroMatrix();
            lhs.aa = zeroMatrix();
            vec6 rhs{};

            mat3 massMat = mat3::diagonal(vec3(body->mass));
            mat3 inertiaWorld = body->getInertiaTensorWorld();
            float invDt2 = 1.0f / (dt * dt);

            lhs.ll += massMat * invDt2;
            lhs.aa += inertiaWorld * invDt2;

            rhs.l = massMat * ((body->position - body->inertialPosition) * invDt2);

            quat qErr = body->orientation * conjugate(body->inertialOrientation);
            vec3 rotErr(qErr.x, qErr.y, qErr.z);
            rotErr = rotErr * 2.0f;
            if (qErr.w < 0.0f) rotErr = -rotErr;
            rhs.a = inertiaWorld * (rotErr * invDt2);

            for (Force* force = body->forces; force != nullptr; force = (force->bodyA == body) ? force->nextA : force->nextB) {
                force->computeConstraint(currentAlpha);
                int rows = force->getRowCount();

                for (int row = 0; row < rows; ++row) {
                    vec3 Jl, Ja;
                    force->computeDerivatives(Jl, Ja, body, row);

                    float lambdaWarm = (force->stiffness[row] == FLT_MAX) ? force->lambda[row] : 0.0f;
                    float desired = force->penalty[row] * force->C[row] + lambdaWarm + force->motor[row];
                    float f = clamp(desired, force->fmin[row], force->fmax[row]);

                    rhs.l += Jl * f;
                    rhs.a += Ja * f;

                    float penalty = force->penalty[row];
                    if (penalty > 0.0f && std::isfinite(penalty)) {
                        lhs.ll += outer(Jl, Jl) * penalty;
                        lhs.la += outer(Jl, Ja) * penalty;
                        lhs.al += outer(Ja, Jl) * penalty;
                        lhs.aa += outer(Ja, Ja) * penalty;

                        if (force->isManifold()) {
                            mat3 invInertiaWorld = body->getInvInertiaTensorWorld();
                            vec3 gyroscopic = abs(cross(Ja, invInertiaWorld * Ja)) * fabsf(f);
                            lhs.aa += mat3::diagonal(gyroscopic);
                        }
                    }
                }
            }

            vec6 dx = solve6x6(lhs, rhs);
            body->position -= dx.l;
            quat dq(dx.a.x, dx.a.y, dx.a.z, 0.0f);
            body->orientation = normalize(body->orientation - (dq * body->orientation) * 0.5f);

            sanitizeVec3(body->position, "position", body->id);
            sanitizeQuat(body->orientation, "orientation", body->id);
        }

        if (it < iterations) {
            for (Force* force = forces; force != nullptr; force = force->next) {
                force->computeConstraint(currentAlpha);
                int rows = force->getRowCount();
                for (int row = 0; row < rows; ++row) {
                    if (force->stiffness[row] != FLT_MAX) {
                        continue;
                    }
                    float lambdaUpdated = clamp(force->penalty[row] * force->C[row] + force->lambda[row],
                                                force->fmin[row], force->fmax[row]);
                    bool active = lambdaUpdated > force->fmin[row] && lambdaUpdated < force->fmax[row];
                    force->lambda[row] = lambdaUpdated;
                    if (active) {
                        force->penalty[row] = min(force->penalty[row] + beta * fabsf(force->C[row]), PENALTY_MAX);
                    }
                }
            }
        }
    }

    // --- 5. Update Velocities ---
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        if (body->invMass <= 0.0f) {
            continue;
        }

        body->linearVelocity = (body->position - body->initialPosition) / dt;
        quat delta_q = body->orientation * conjugate(body->initialOrientation);
        vec3 angVel(delta_q.x, delta_q.y, delta_q.z);
        angVel = angVel * (2.0f / dt);
        if (delta_q.w < 0.0f) angVel = -angVel;
        body->angularVelocity = angVel;

        sanitizeVec3(body->linearVelocity, "linear velocity", body->id);
        sanitizeVec3(body->angularVelocity, "angular velocity", body->id);

        float linearSpeed = length(body->linearVelocity);
        float angularSpeed = length(body->angularVelocity);
        stats.maxLinearSpeed = max(stats.maxLinearSpeed, linearSpeed);
        stats.maxAngularSpeed = max(stats.maxAngularSpeed, angularSpeed);
    }

    // --- 6. Diagnostics Collection ---
    for (Force* force = forces; force != nullptr; force = force->next) {
        if (!force->isManifold()) {
            continue;
        }
        Manifold* manifold = static_cast<Manifold*>(force);
        stats.activeManifolds++;
        stats.activeContacts += manifold->numContacts;

        for (int i = 0; i < manifold->numContacts; ++i) {
            const Manifold::Contact& contact = manifold->contacts[i];
            vec3 world_rA = rotate(manifold->bodyA->orientation, contact.rA);
            vec3 world_rB = rotate(manifold->bodyB->orientation, contact.rB);
            vec3 pA = manifold->bodyA->position + world_rA;
            vec3 pB = manifold->bodyB->position + world_rB;

            float separation = dot(pA - pB, contact.normal);
            float penetration = max(0.0f, -separation);
            float violation = max(0.0f, PENETRATION_SLOP - separation);

            stats.maxPenetration = max(stats.maxPenetration, penetration);
            stats.maxConstraintViolation = max(stats.maxConstraintViolation, violation);
            stats.maxNormalImpulse = max(stats.maxNormalImpulse, fabsf(force->lambda[i * 3 + 0]));
        }
    }

    lastDiagnostics = stats;

    if (enableDiagnostics) {
        int frequency = logFrequency > 0 ? logFrequency : 1;
        if (stepIndex % frequency == 0) {
            std::printf("[Physics] step %d | manifolds: %d | contacts: %d | dyn bodies: %d | maxPen: %.6f | maxDrift: %.6f | maxLin: %.3f | maxAng: %.3f | maxLambda: %.3f\n",
                        stepIndex,
                        stats.activeManifolds,
                        stats.activeContacts,
                        stats.dynamicBodies,
                        stats.maxPenetration,
                        stats.maxConstraintViolation,
                        stats.maxLinearSpeed,
                        stats.maxAngularSpeed,
                        stats.maxNormalImpulse);
        }
    }
}

void Solver::draw()
{
    for (Rigid* body = bodies; body != nullptr; body = body->next) body->draw();
    for (Force* force = forces; force != nullptr; force = force->next) force->draw();
}
