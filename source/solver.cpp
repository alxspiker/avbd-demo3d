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

constexpr float MANIFOLD_PENALTY_CAP = 2000000.0f;

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

static float rowPenaltyGain(const Force* force, int row, float beta)
{
    // Blend linear and angular penalty ramping (inspired by the original
    // solver split into betaLin/betaAng) while preserving the current API.
    constexpr float angularBetaScale = 0.01f;
    constexpr float epsilon = 1.0e-8f;

    float linearWeight = 0.0f;
    float angularWeight = 0.0f;

    auto accumulateWeights = [&](const Rigid* body) {
        if (body == nullptr) {
            return;
        }
        vec3 Jl, Ja;
        force->computeDerivatives(Jl, Ja, body, row);
        linearWeight += lengthSq(Jl);
        angularWeight += lengthSq(Ja);
    };

    accumulateWeights(force->bodyA);
    accumulateWeights(force->bodyB);

    float totalWeight = linearWeight + angularWeight;
    if (totalWeight < epsilon) {
        return beta;
    }

    float betaLinear = beta;
    float betaAngular = beta * angularBetaScale;
    return (betaLinear * linearWeight + betaAngular * angularWeight) / totalWeight;
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

Rigid* Solver::pick(const vec3& origin, const vec3& dir, vec3& local)
{
    const float epsilon = 1.0e-6f;
    float bestT = FLT_MAX;
    Rigid* bestBody = nullptr;
    vec3 bestLocal;

    vec3 rayDir = dir;
    float dirLenSq = lengthSq(rayDir);
    if (dirLenSq < epsilon) {
        return nullptr;
    }
    rayDir = rayDir / sqrtf(dirLenSq);

    // Ray-cast against each dynamic OBB by transforming the ray into local
    // space and using slab intersection.
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        if (body->invMass <= 0.0f) {
            continue;
        }

        quat invRot = conjugate(body->orientation);
        vec3 localOrigin = rotate(invRot, origin - body->position);
        vec3 localDir = rotate(invRot, rayDir);
        vec3 half = body->size * 0.5f;

        float tEnter = 0.0f;
        float tExit = FLT_MAX;
        bool hit = true;

        for (int axis = 0; axis < 3; ++axis) {
            float o = localOrigin[axis];
            float d = localDir[axis];
            float minB = -half[axis];
            float maxB = half[axis];

            if (fabsf(d) < epsilon) {
                if (o < minB || o > maxB) {
                    hit = false;
                    break;
                }
                continue;
            }

            float invD = 1.0f / d;
            float t0 = (minB - o) * invD;
            float t1 = (maxB - o) * invD;
            if (t0 > t1) {
                float tmp = t0;
                t0 = t1;
                t1 = tmp;
            }

            tEnter = max(tEnter, t0);
            tExit = min(tExit, t1);
            if (tEnter > tExit) {
                hit = false;
                break;
            }
        }

        if (!hit) {
            continue;
        }

        float tHit = (tEnter >= 0.0f) ? tEnter : tExit;
        if (tHit < 0.0f) {
            continue;
        }

        if (tHit < bestT) {
            bestT = tHit;
            bestBody = body;
            bestLocal = localOrigin + localDir * tHit;
        }
    }

    if (bestBody == nullptr) {
        return nullptr;
    }

    local = bestLocal;
    return bestBody;
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
    alpha = 0.95f;
    beta = 100000.0f;
    gamma = 0.99f;
    postStabilize = false;
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
                        float betaRow = rowPenaltyGain(force, row, beta);
                        float penaltyCap = force->isManifold() ? MANIFOLD_PENALTY_CAP : PENALTY_MAX;
                        force->penalty[row] = min(force->penalty[row] + betaRow * fabsf(force->C[row]), penaltyCap);
                    }
                }
            }
        }
    }

        // --- 5. Update Velocities with Damping ---
    const float linearDamping = 0.995f;  // Keep responsive free-fall while damping residual jitter
    const float angularDamping = 0.97f;  // Preserve rotational stability without overdamping
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        if (body->invMass <= 0.0f) {
            continue;
        }

        // Keep previous step velocities for adaptive warm-start weighting.
        body->prevLinearVelocity = body->linearVelocity;
        body->prevAngularVelocity = body->angularVelocity;

        body->linearVelocity = (body->position - body->initialPosition) / dt;
        quat delta_q = body->orientation * conjugate(body->initialOrientation);
        vec3 angVel(delta_q.x, delta_q.y, delta_q.z);
        angVel = angVel * (2.0f / dt);
        if (delta_q.w < 0.0f) angVel = -angVel;
        body->angularVelocity = angVel;

                // Apply damping to dissipate energy and stabilize resting contacts
        body->linearVelocity = body->linearVelocity * linearDamping;
        body->angularVelocity = body->angularVelocity * angularDamping;
        
        // Sleep threshold: if velocity is very low, set to zero to prevent drift
        float linSpeed = length(body->linearVelocity);
        float angSpeed = length(body->angularVelocity);
        // Sleep disabled - was cutting off support
        // Sleep disabled

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
