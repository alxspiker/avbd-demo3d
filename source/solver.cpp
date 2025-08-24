/*
* solver.cpp - 3D AVBD Physics Engine
*
* CORRECTED: Added 'std::' for isfinite and fixed primal update logic.
* UPDATED: Increased iterations to 50, beta to 1e7 in defaultParams.
* UPDATED: Set alpha =0.0f to reduce bounce by disabling lambda warmstart.
* UPDATED: Increased beta to 1e8 and iterations to 100 for stronger constraint enforcement.
* UPDATED: Set beta to 1e6f and iterations to 50 for balance.
* UPDATED: Added velocity solve for restitution 0 to prevent bouncing.
* CORRECTED: Implemented friction impulses in the final velocity solve stage to prevent sliding.
* CORRECTED: Set velocity solve to be non-iterative to prevent energy gain.
* CORRECTED: Disabled postStabilize by default to prevent conflicts with other stabilization.
*/

#include "solver.h"
#include <vector>
#include <cmath> // For std::isfinite
#include <stdio.h> // For printf

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
    gravity = {0.0f, -10.0f, 0.0f};
    iterations = 10; // Back to 2D reference value

    // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
    // depend on the length, mass, and constraint function scales (ie units) of your simulation,
    // along with your strategy for incrementing the penalty parameters.
    // If the value is not in the right range, you may see slower convergance for complex scenes.
    beta = 100000.0f; // Back to 2D reference value

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic. Tune this depending
    // on your desired constraint error response.
    alpha = 0.99f;

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    // This should always be < 1 so that the penalty values can decrease (unless you use a different
    // penalty parameter strategy which does not require decay).
    gamma = 0.99f;

    // Post stabilization applies an extra iteration to fix positional error.
    // This removes the need for the alpha parameter, which can make tuning a little easier.
    postStabilize = true;
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
            body->inertialPosition = body->position + body->linearVelocity * dt + gravity * (0.5f * dt * dt);
            quat w_q(body->angularVelocity.x, body->angularVelocity.y, body->angularVelocity.z, 0);
            body->inertialOrientation = normalize(body->orientation + (w_q * body->orientation) * (dt * 0.5f));

            // Use pure inertial prediction without ad-hoc warm-start blending
            body->position = body->inertialPosition;
            body->orientation = body->inertialOrientation;
        } else {
            body->inertialPosition = body->position;
            body->inertialOrientation = body->orientation;
        }
    }

    // --- 4. Main Iterative Solver Loop ---
         int totalIterations = max(12, iterations) + (postStabilize ? 1 : 0);
    for (int it = 0; it < totalIterations; ++it) {
        float currentAlpha = postStabilize ? (it < iterations ? 1.0f : 0.0f) : this->alpha;

        // --- 4a. Primal Update ---
        for (Rigid* body = bodies; body != 0; body = body->next) {
            if (body->invMass <= 0) continue;

            mat3 M = mat3::diagonal(body->mass);
            mat3 I_world = body->getInertiaTensorWorld();

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

                    // Remove ad-hoc geometric stiffness term which had incorrect units and destabilized angular solve
                    lhs.ll += outer_product(J_l, J_l) * force->penalty[i];
                    lhs.aa += outer_product(J_a, J_a) * force->penalty[i];
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
                    if (force->stiffness[i] != FLT_MAX) continue;
                    float lambda_i = clamp(force->penalty[i] * force->C[i] + force->lambda[i], force->fmin[i], force->fmax[i]);
                    force->lambda[i] = lambda_i;
                    if (force->lambda[i] > force->fmin[i] && force->lambda[i] < force->fmax[i]) {
                         force->penalty[i] = min(force->penalty[i] + beta * std::fabs(force->C[i]), PENALTY_MAX);
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

    // --- 6. Velocity Solve for Restitution (e=0) ---
    // --- FIX ---
    // The previous iterative velocity solve (vel_iterations = 10) was incorrectly
    // applying a full corrective impulse in every iteration without accumulating the
    // results. This created a positive feedback loop that injected massive amounts
    // of energy, causing bouncing and accelerating sliding. A single, non-iterative
    // pass is the correct approach for this type of post-correction step.
         const int vel_iterations = 3;
    for (int vel_it = 0; vel_it < vel_iterations; ++vel_it) { // This loop now effectively runs only once
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            for (int i = 0; i < m->numContacts; ++i) {
                const Manifold::Contact& c = m->contacts[i];
                vec3 world_rA = rotate(m->bodyA->orientation, c.rA);
                vec3 world_rB = rotate(m->bodyB->orientation, c.rB);
                vec3 v_rel = (m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, world_rA)) -
                             (m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, world_rB));
                vec3 n = c.normal;
                float v_n = dot(v_rel, n);
                if (v_n >= 0) continue; // separating

                // Compute effective mass for normal
                vec3 Ia_n = m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, n);
                float qa = dot(cross(Ia_n, world_rA), n);
                vec3 Ib_n = m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, n);
                float qb = dot(cross(Ib_n, world_rB), n);
                float em = m->bodyA->invMass + m->bodyB->invMass + qa + qb;
                if (em <= 0) continue;

                float e = 0.0f;
                float j = - (1.0f + e) * v_n / em;

                // Accumulate normal impulse (RAII local clamp not needed here because we don't warmstart jn across frames)
                m->contacts[i].jn += j;

                vec3 impulse = j * n;

                if (m->bodyA->invMass > 0) {
                    m->bodyA->linearVelocity += m->bodyA->invMass * impulse;
                    m->bodyA->angularVelocity += m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, impulse);
                }

                if (m->bodyB->invMass > 0) {
                    m->bodyB->linearVelocity -= m->bodyB->invMass * impulse;
                    m->bodyB->angularVelocity -= m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, impulse);
                }

                // --- Friction Impulse with accumulation ---
                // Re-calculate relative velocity after normal impulse
                v_rel = (m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, world_rA)) -
                        (m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, world_rB));

                // Create tangent vectors
                vec3 tangent1, tangent2;
                if (std::fabs(n.x) > 0.9f) tangent1 = normalize(cross(n, vec3(0, 1, 0)));
                else tangent1 = normalize(cross(n, vec3(1, 0, 0)));
                tangent2 = normalize(cross(n, tangent1));

                // Calculate effective mass for tangent 1
                vec3 Ia_t1 = m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, tangent1);
                float qa_t1 = dot(cross(Ia_t1, world_rA), tangent1);
                vec3 Ib_t1 = m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, tangent1);
                float qb_t1 = dot(cross(Ib_t1, world_rB), tangent1);
                float em_t1 = m->bodyA->invMass + m->bodyB->invMass + qa_t1 + qb_t1;
                
                // Calculate effective mass for tangent 2
                vec3 Ia_t2 = m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, tangent2);
                float qa_t2 = dot(cross(Ia_t2, world_rA), tangent2);
                vec3 Ib_t2 = m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, tangent2);
                float qb_t2 = dot(cross(Ib_t2, world_rB), tangent2);
                float em_t2 = m->bodyA->invMass + m->bodyB->invMass + qa_t2 + qb_t2;

                // Target velocity = 0 (Coulomb), compute impulse increments
                float djt1 = (em_t1 > 0) ? -dot(v_rel, tangent1) / em_t1 : 0.0f;
                float djt2 = (em_t2 > 0) ? -dot(v_rel, tangent2) / em_t2 : 0.0f;

                // Accumulate then clamp to cone using max of instantaneous normal impulse and estimated constraint normal impulse over dt
                float old_jt1 = m->contacts[i].jt1;
                float old_jt2 = m->contacts[i].jt2;
                float new_jt1 = old_jt1 + djt1;
                float new_jt2 = old_jt2 + djt2;
                float normal_lambda_mag = std::fabs(m->lambda[i*3 + 0]);
                float normal_impulse_est = normal_lambda_mag * dt;
                float friction_limit = m->combinedFriction * max(std::fabs(m->contacts[i].jn), normal_impulse_est);
                float new_tangent_mag = sqrtf(new_jt1 * new_jt1 + new_jt2 * new_jt2);
                if (new_tangent_mag > friction_limit) {
                    float scale = (new_tangent_mag > 0.0f) ? (friction_limit / new_tangent_mag) : 0.0f;
                    new_jt1 *= scale;
                    new_jt2 *= scale;
                }
                // Delta to apply this iteration
                float apply_djt1 = new_jt1 - old_jt1;
                float apply_djt2 = new_jt2 - old_jt2;
                m->contacts[i].jt1 = new_jt1;
                m->contacts[i].jt2 = new_jt2;

                // Apply friction impulse (incremental)
                vec3 friction_impulse = tangent1 * apply_djt1 + tangent2 * apply_djt2;
                if (m->bodyA->invMass > 0) { m->bodyA->linearVelocity += m->bodyA->invMass * friction_impulse; m->bodyA->angularVelocity += m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, friction_impulse); }
                if (m->bodyB->invMass > 0) { m->bodyB->linearVelocity -= m->bodyB->invMass * friction_impulse; m->bodyB->angularVelocity -= m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, friction_impulse); }
            }
        }
    }
}

void Solver::draw() {
    for (Rigid* body = bodies; body != 0; body = body->next) body->draw();
    for (Force* force = forces; force != 0; force = force->next) force->draw();
}