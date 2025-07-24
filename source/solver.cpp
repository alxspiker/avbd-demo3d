/*
* solver.cpp - 3D AVBD Physics Engine
*
* CORRECTED: Added 'std::' for isfinite and fixed primal update logic.
*/

#include "solver.h"
#include <vector>
#include <cmath> // For std::isfinite

// Helper structures for the 6-DOF solver
struct vec6 { vec3 l; vec3 a; };
struct mat66 { mat3 ll, la, al, aa; };

// Solve the 6x6 system. We decouple the linear and angular parts.
vec6 solve6x6(const mat66& A, const vec6& b) {
    vec3 linear_part = solve(A.ll, b.l);
    vec3 angular_part = solve(A.aa, b.a);
    return {linear_part, angular_part};
}

Solver::Solver() : bodies(0), forces(0) {
    defaultParams();
}

Solver::~Solver() {
    clear();
}

void Solver::clear() {
    while (forces) delete forces;
    while (bodies) delete bodies;
}

void Solver::defaultParams() {
    dt = 1.0f / 60.0f;
    gravity = {0.0f, -9.81f, 0.0f};
    iterations = 10;
    beta = 10000.0f;
    alpha = 0.2f;
    gamma = 0.9f;
    postStabilize = false;
}

void Solver::step() {
    // --- 1. Broadphase ---
    for (Rigid* bodyA = bodies; bodyA != 0; bodyA = bodyA->next) {
        for (Rigid* bodyB = bodyA->next; bodyB != 0; bodyB = bodyB->next) {
            vec3 dp = bodyA->position - bodyB->position;
            float r = bodyA->radius + bodyB->radius;
            if (dot(dp, dp) <= r * r && !bodyA->isConstrainedTo(bodyB)) {
                new Manifold(this, bodyA, bodyB);
            }
        }
    }

    // --- 2. Initialize and Warmstart Forces ---
    for (Force* force = forces; force != 0; ) {
        if (!force->initialize()) {
            Force* next = force->next;
            delete force;
            force = next;
        } else {
            for (int i = 0; i < force->getRowCount(); ++i) {
                if (postStabilize) {
                    force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
                } else {
                    force->lambda[i] *= alpha * gamma;
                    force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
                }
                force->penalty[i] = min(force->penalty[i], force->stiffness[i] == 0 ? FLT_MAX : force->stiffness[i]);
            }
            force = force->next;
        }
    }
    
    // --- 3. Predict Body States ---
    for (Rigid* body = bodies; body != 0; body = body->next) {
        body->initialPosition = body->position;
        body->initialOrientation = body->orientation;

        if (body->invMass > 0) {
            // Compute inertial state
            body->inertialPosition = body->position + body->linearVelocity * dt + gravity * (dt * dt);
            quat w_q(body->angularVelocity.x, body->angularVelocity.y, body->angularVelocity.z, 0);
            body->inertialOrientation = normalize(body->orientation + (w_q * body->orientation) * (dt * 0.5f));

            // Adaptive warm-starting
            vec3 accel = (body->linearVelocity - body->prevLinearVelocity) / dt;
            float accelExt = dot(accel, normalize(gravity));
            float accelWeight = clamp(accelExt / length(gravity), 0.0f, 1.0f);
            if (!std::isfinite(accelWeight)) accelWeight = 0.0f; // CORRECTED

            // Update current state to warm-started prediction
            body->position += body->linearVelocity * dt + gravity * (accelWeight * dt * dt);
            body->orientation = body->inertialOrientation;
        } else {
            body->inertialPosition = body->position;
            body->inertialOrientation = body->orientation;
        }
    }

    // --- 4. Main Iterative Solver Loop ---
    int totalIterations = iterations + (postStabilize ? 1 : 0);
    for (int it = 0; it < totalIterations; ++it) {
        float currentAlpha = postStabilize ? (it < iterations ? 1.0f : 0.0f) : this->alpha;

        // --- 4a. Primal Update ---
        for (Rigid* body = bodies; body != 0; body = body->next) {
            if (body->invMass <= 0) continue;

            mat3 M = mat3::diagonal(body->mass);
            mat3 I_world = transpose(body->getInvInertiaTensorWorld()); // get I, not I^-1

            mat66 lhs = {};
            vec6 rhs = {};
            
            lhs.ll = M * (1.0f / (dt * dt));
            lhs.aa = I_world * (1.0f / (dt * dt));
            
            rhs.l = (body->position - body->inertialPosition) * (body->mass / (dt * dt));
            
            quat q_err = body->orientation * conjugate(body->inertialOrientation);
            vec3 rot_err_vec = vec3(q_err.x, q_err.y, q_err.z) * 2.0f;
            if(q_err.w < 0) rot_err_vec = -rot_err_vec;
            rhs.a = I_world * rot_err_vec * (1.0f / (dt*dt));

            // Accumulate forces
            for (Force* force = body->forces; force != 0; force = (force->bodyA == body) ? force->nextA : force->nextB) {
                force->computeConstraint(currentAlpha);
                for (int i = 0; i < force->getRowCount(); ++i) {
                    float lambda_i = (force->stiffness[i] == FLT_MAX) ? force->lambda[i] : 0.0f;
                    float f = clamp(force->penalty[i] * force->C[i] + lambda_i + force->motor[i], force->fmin[i], force->fmax[i]);

                    vec3 J_l, J_a;
                    force->computeDerivatives(J_l, J_a, body, i);
                    
                    rhs.l += J_l * f;
                    rhs.a += J_a * f;

                    mat3 G_a = mat3::diagonal(abs(cross(J_a, I_world * J_a)) * f);
                    
                    lhs.ll += outer_product(J_l, J_l) * force->penalty[i];
                    lhs.aa += outer_product(J_a, J_a) * force->penalty[i] + G_a;
                }
            }

            // Solve and apply update
            vec6 dx = solve6x6(lhs, rhs);
            body->position -= dx.l;
            quat dq(dx.a.x, dx.a.y, dx.a.z, 0);
            body->orientation = normalize(body->orientation - (dq * body->orientation) * 0.5f);
        }

        // --- 4b. Dual Update ---
        if (it < iterations) {
            for (Force* force = forces; force != 0; force = force->next) {
                force->computeConstraint(currentAlpha);
                for (int i = 0; i < force->getRowCount(); ++i) {
                    if(force->stiffness[i] != FLT_MAX) continue;
                    float lambda_i = clamp(force->penalty[i] * force->C[i] + force->lambda[i], force->fmin[i], force->fmax[i]);
                    force->lambda[i] = lambda_i;
                    if (force->lambda[i] > force->fmin[i] && force->lambda[i] < force->fmax[i]) {
                         force->penalty[i] = min(force->penalty[i] + beta * abs(force->C[i]), PENALTY_MAX);
                    }
                }
            }
        }
    }

    // --- 5. Update Velocities ---
    for (Rigid* body = bodies; body != 0; body = body->next) {
        body->prevLinearVelocity = body->linearVelocity;
        body->prevAngularVelocity = body->angularVelocity;
        if (body->invMass <= 0) continue;

        body->linearVelocity = (body->position - body->initialPosition) / dt;
        quat delta_q = body->orientation * conjugate(body->initialOrientation);
        body->angularVelocity = vec3(delta_q.x, delta_q.y, delta_q.z) * (2.0f / dt);
        if (delta_q.w < 0) body->angularVelocity = -body->angularVelocity;
    }
}

void Solver::draw() {
    for (Rigid* body = bodies; body != 0; body = body->next) body->draw();
    for (Force* force = forces; force != 0; force = force->next) force->draw();
}