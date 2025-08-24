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
#include <algorithm> // for std::sort
#include <cstdio>

bool solver_should_trace(const Solver* s) {
    if (!s) return false;
    if (s->debugAllConstraints) return true;
    if (s->debugSingleFrame >= 0 && s->frameIndex == s->debugSingleFrame) return true;
    if (s->debugRangeStart >= 0 && s->debugRangeEnd >= s->debugRangeStart && s->frameIndex >= s->debugRangeStart && s->frameIndex <= s->debugRangeEnd) return true;
    return false;
}

// Local helper (GLSL-style mix) since maths.h does not define it
static inline vec3 mix(const vec3& a, const vec3& b, float t) { return a * (1.0f - t) + b * t; }

// Helper structures for the 6-DOF solver
struct vec6 { vec3 l; vec3 a; };
struct mat66 { mat3 ll, la, al, aa; };

// Solve the 6x6 system properly handling coupling terms
vec6 solve6x6(const mat66& A, const vec6& b) {
    // For small systems, it's often more stable to solve the coupled system
    // by block elimination rather than decoupling
    
    // Check if coupling terms are significant
    float ll_norm = length(A.ll * vec3(1,1,1));
    float aa_norm = length(A.aa * vec3(1,1,1));
    float la_norm = length(A.la * vec3(1,1,1));
    float al_norm = length(A.al * vec3(1,1,1));
    
    float coupling_ratio = (la_norm + al_norm) / (ll_norm + aa_norm + 1e-10f);
    
    if (coupling_ratio < 0.1f) {
        // Weak coupling, safe to decouple
        vec3 linear_part = solve(A.ll, b.l);
        vec3 angular_part = solve(A.aa, b.a);
        return {linear_part, angular_part};
    } else {
        // Strong coupling, solve with block elimination
        // Solve: [A.ll  A.la] [x_l] = [b.l]
        //        [A.al  A.aa] [x_a]   [b.a]
        
        // Using Schur complement: x_a = (A.aa - A.al * A.ll^-1 * A.la)^-1 * (b.a - A.al * A.ll^-1 * b.l)
        mat3 A_ll_inv;
        if (!invert(A.ll, A_ll_inv)) {
            // Fallback to decoupled solution if inversion fails
            vec3 linear_part = solve(A.ll, b.l);
            vec3 angular_part = solve(A.aa, b.a);
            return {linear_part, angular_part};
        }
        
        mat3 schur = A.aa - A.al * A_ll_inv * A.la;
        vec3 rhs_schur = b.a - A.al * A_ll_inv * b.l;
        
        vec3 x_a = solve(schur, rhs_schur);
        vec3 x_l = A_ll_inv * (b.l - A.la * x_a);
        
        return {x_l, x_a};
    }
}

Solver::Solver() : bodies(0), forces(0), frameIndex(0) {
    defaultParams();
}

Solver::~Solver() {
    clear();
    if (csvLogFile) {
        fclose(csvLogFile);
        csvLogFile = nullptr;
    }
}

void Solver::clear() {
    while (forces) delete forces;
    while (bodies) delete bodies;
}

void Solver::defaultParams() {
    dt = 1.0f / 60.0f;
    gravity = {0.0f, -10.0f, 0.0f};
    iterations = 10; // Fewer iterations mitigate over-correction energy build-up

    // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
    // depend on the length, mass, and constraint function scales (ie units) of your simulation,
    // along with your strategy for incrementing the penalty parameters.
    // If the value is not in the right range, you may see slower convergance for complex scenes.
    beta = 200.0f; // Further reduced: slower penalty growth for stability

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic. Tune this depending
    // on your desired constraint error response.
    alpha = 0.6f; // Slightly lower to reduce aggressive accumulation

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    // This should always be < 1 so that the penalty values can decrease (unless you use a different
    // penalty parameter strategy which does not require decay).
    gamma = 0.9f; // More aggressive decay for better stability

    // Post stabilization applies an extra iteration to fix positional error.
    // This removes the need for the alpha parameter, which can make tuning a little easier.
    postStabilize = false; // Keep disabled to prevent conflicts
}

void Solver::apply2DParamPreset() {
    // Mirror core parameter defaults from the original 2D demo for comparison
    // NOTE: We intentionally do NOT touch dt or gravity; those remain as currently configured.
    iterations = 10;
    beta = 100000.0f;   // Faster penalty growth like 2D sample
    alpha = 0.99f;      // Strong warmstart retention
    gamma = 0.99f;      // Slow decay of penalty / lambdas
    postStabilize = true; // Enable post stabilization pass
}

void Solver::step() {
    frameIndex++;
    // Enforce fixed iteration budget (user requirement): clamp to 10 if altered externally
    if (iterations != 10) iterations = 10;
    // Reset per-frame diagnostics counters
    diagAxisClampCount = 0;
    diagSpikeProjCount = 0;
    diagConstraintComputeCalls = 0;
    diagConstraintHeavyUpdateCalls = 0;
    diagManifoldCount = 0;
    diagManifoldsDropped = 0;
    diagManifoldsResurrected = 0;
    diagContactCount = 0; // reset total contacts counter
    // --- Early Phase Pre-Separation (vertical) ---
    // To prevent extremely deep initial interpenetrations from ballistic descent, gently separate
    // pairs whose vertical overlap greatly exceeds a safe early limit before collision detection.
    if (frameIndex < 120) {
        const float EARLY_VERTICAL_OVERLAP_LIMIT = 0.15f; // meters
        for (Rigid* A = bodies; A; A = A->next) {
            for (Rigid* B = A->next; B; B = B->next) {
                if (A->isConstrainedTo(B)) continue;
                vec3 d = B->position - A->position;
                vec3 halfA = A->size * 0.5f;
                vec3 halfB = B->size * 0.5f;
                float allowedX = (halfA.x + halfB.x) * 0.6f;
                float allowedZ = (halfA.z + halfB.z) * 0.6f;
                if (fabsf(d.x) > allowedX || fabsf(d.z) > allowedZ) continue;
                float totalHalfY = halfA.y + halfB.y;
                float overlapY = totalHalfY - fabsf(d.y);
                if (overlapY <= EARLY_VERTICAL_OVERLAP_LIMIT) continue;
                float excess = overlapY - EARLY_VERTICAL_OVERLAP_LIMIT;
                // Move only the upper dynamic body upward (never push any body downward)
                if (d.y >= 0.0f) { // B above A
                    if (B->invMass > 0) B->position.y += excess * 0.5f; // gentle half correction
                } else { // A above B
                    if (A->invMass > 0) A->position.y += excess * 0.5f;
                }
            }
        }
        // Safety: lift stack if any dynamic body sinks far below initial baseline (prevents runaway tunneling)
        float minY = 1e9f;
        for (Rigid* b = bodies; b; b = b->next) if (b->invMass > 0 && b->position.y < minY) minY = b->position.y;
        if (minY < -0.2f) {
            float lift = -minY; // bring lowest to y=0
            for (Rigid* b = bodies; b; b = b->next) if (b->invMass > 0) b->position.y += lift;
        }
    }
    // Global early-phase downward velocity clamp to reduce high-speed impacts causing deep initial penetration
    if (frameIndex < 120) {
        const float MAX_EARLY_DOWN_V = -6.0f; // m/s cap
        for (Rigid* b = bodies; b; b = b->next) {
            if (b->invMass <= 0) continue;
            if (b->linearVelocity.y < MAX_EARLY_DOWN_V) b->linearVelocity.y = MAX_EARLY_DOWN_V;
        }
    }
    // --- 1. Broadphase (AABB overlap) ---
    // Previous sphere-distance broadphase missed stacked boxes until deep overlap.
    // Broaden broadphase margin slightly to reduce one-frame misses that destabilize stacks.
    const float broadphaseMargin = COLLISION_MARGIN * 3.0f + CONTACT_PERSISTENCE_DISTANCE * 1.2f;
    for (Rigid* bodyA = bodies; bodyA != 0; bodyA = bodyA->next) {
        vec3 halfA = bodyA->size * 0.5f;
        for (Rigid* bodyB = bodyA->next; bodyB != 0; bodyB = bodyB->next) {
            if (bodyA->isConstrainedTo(bodyB)) continue;
            vec3 halfB = bodyB->size * 0.5f;
            vec3 d = bodyB->position - bodyA->position;
            if (fabsf(d.x) <= (halfA.x + halfB.x + broadphaseMargin) &&
                fabsf(d.y) <= (halfA.y + halfB.y + broadphaseMargin) &&
                fabsf(d.z) <= (halfA.z + halfB.z + broadphaseMargin)) {
                new Manifold(this, bodyA, bodyB);
            }
        }
    }

    // --- 2. Initialize and Warmstart Forces ---
    for (Force* force = forces; force != 0; ) {
        if (!force->initialize()) {
            // Guard: if this was a manifold, apply hysteresis pruning conditions before deletion
            if (force->isManifold()) {
                Manifold* mdrop = (Manifold*)force;
                bool allowDrop = true;
                // Prevent drop if manifold recently had shallow penetration memory and stack still moving
                float topVy = 0.0f; int topCt=0; // compute quick avg top vy (reuse method later, cheap for few bodies)
                for (Rigid* bTmp = bodies; bTmp; bTmp = bTmp->next) { if (bTmp->invMass>0) { topVy += bTmp->linearVelocity.y; topCt++; } }
                if (topCt>0) topVy/=topCt;
                if (mdrop->framesAlive < 30) allowDrop = false; // grace period
                if (mdrop->framesSincePenetration < 75) allowDrop = false; // keep while recently penetrating or shallow
                // Single re-collision attempt with slight positional inflation before dropping
                if (allowDrop) {
                    vec3 halfA = mdrop->bodyA->size * 0.5f;
                    vec3 halfB = mdrop->bodyB->size * 0.5f;
                    vec3 d = mdrop->bodyB->position - mdrop->bodyA->position;
                    float overlapX = (halfA.x + halfB.x) - fabsf(d.x);
                    float overlapZ = (halfA.z + halfB.z) - fabsf(d.z);
                    float totalHalfY = halfA.y + halfB.y;
                    float gapY = fabsf(d.y) - totalHalfY;
                    if (overlapX > -COLLISION_MARGIN*0.5f && overlapZ > -COLLISION_MARGIN*0.5f && gapY < COLLISION_MARGIN*0.5f) {
                        Manifold::Contact temp[4];
                        int newCount = Manifold::collide(mdrop->bodyA, mdrop->bodyB, temp, false);
                        if (newCount > 0) {
                            for (int c=0;c<newCount && c<4;++c) mdrop->contacts[c] = temp[c];
                            mdrop->numContacts = newCount;
                            allowDrop = false; // resurrect via immediate fresh collision
                        }
                    }
                }
                if (allowDrop && mdrop->resurrectedThisFrame) allowDrop = false; // keep at least a frame after resurrection
                if (!allowDrop) {
                    // Preserve full contact set (do not clamp numContacts) and decay penalties slightly.
                    for (int i=0;i<mdrop->getRowCount();++i){ mdrop->penalty[i]=clamp(mdrop->penalty[i]*gamma, PENALTY_MIN, PENALTY_MAX);} 
                    force = force->next; // continue without deleting
                    continue;
                }
                diagManifoldsDropped++;
            }
            Force* next = force->next;
            delete force;
            force = next;
        } else {
            if (force->isManifold()) {
                Manifold* m = (Manifold*)force;
                m->computeCallsThisFrame = 0;
                diagManifoldCount++;
                diagContactCount += m->numContacts; // accumulate contact total
            }
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
        body->groundedContacts = 0; // reset grounded contact count
        body->supportLoad = 0.0f;
        body->isBottomSupport = false;
        body->initialPosition = body->position;
        body->initialOrientation = body->orientation;
        body->frameLinearCorrectionAccum = 0.0f;
        body->frameAngularCorrectionAccum = 0.0f;

        // Store pre-solve velocities (before positional corrections) so we can decouple
        // velocity integration from large positional corrections performed by the solver.
        body->preSolveLinearVelocity = body->linearVelocity;
        body->preSolveAngularVelocity = body->angularVelocity;

        if (body->invMass > 0) {
            // Compute inertial state
            // Use standard ballistic prediction: x' = x + v*dt + 0.5*g*dt^2
            body->inertialPosition = body->position + body->linearVelocity * dt + gravity * (0.5f * dt * dt);
            quat w_q(body->angularVelocity.x, body->angularVelocity.y, body->angularVelocity.z, 0);
            body->inertialOrientation = normalize(body->orientation + (w_q * body->orientation) * (dt * 0.5f));
            // Start the solver from the inertial prediction
            body->position = body->inertialPosition;
            body->orientation = body->inertialOrientation;
        } else {
            body->inertialPosition = body->position;
            body->inertialOrientation = body->orientation;
        }
    }

    // --- 3b. One-time temporal constraint state update (heavy) ---
    // Run heavy manifold state mutation once per frame (penetration clamping, stability, spike detection)
    if (!classicMode) {
        debugPhase = (char*)"update";
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            m->updateConstraintState(this->alpha);
        }
    }
    // Skip remaining heuristic phases if in classic mode
    if (classicMode) goto skip_heuristics;

    // Identify bottom support layer (lowest quartile by Y among dynamic bodies) after heavy update
    {
        std::vector<Rigid*> dynamic;
        for (Rigid* b = bodies; b; b = b->next) if (b->invMass > 0) dynamic.push_back(b);
        if (!dynamic.empty()) {
            std::sort(dynamic.begin(), dynamic.end(), [](Rigid* a, Rigid* b){return a->position.y < b->position.y;});
            size_t layerCount = std::max<size_t>(1, dynamic.size()/4);
            for (size_t i=0;i<layerCount;++i) dynamic[i]->isBottomSupport = true;
        }
    }


    // --- 3c. Spike Projection Pre-Pass ---
    // Heavy temporal contact state already updated in 3b. We only read existing spike flags / raw penetration here.
    {
        float PROJ_TRIGGER_DEPTH = 0.14f; // trigger when raw depth exceeds 14cm (beyond typical cap)
        if (frameIndex > 300) PROJ_TRIGGER_DEPTH = 0.045f; // tighten to 4.5cm after settling window
        debugPhase = "projection";
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            diagCallsProjection += m->numContacts; // one pass per manifold
            for (int ci=0; ci < m->numContacts; ++ci) {
                Manifold::Contact &c = m->contacts[ci];
                float rawPen = c.penetration;
                if (!(c.spikeThisFrame || rawPen > PROJ_TRIGGER_DEPTH)) continue;
                float excess = rawPen - PROJ_TRIGGER_DEPTH;
                if (excess <= 0) excess = rawPen * 0.25f; // mild separation for spike
                float push = excess * 0.3f; // project a fraction to avoid teleporting
                if (push <= 0) continue;
                vec3 n = c.normal; // points from bodyB to bodyA per convention
                float invMassA = m->bodyA->invMass;
                float invMassB = m->bodyB->invMass;
                float invSum = invMassA + invMassB;
                if (invSum <= 0) continue;
                float aShare = (invMassA > 0) ? (invMassA / invSum) : 0.0f;
                float bShare = (invMassB > 0) ? (invMassB / invSum) : 0.0f;
                if (invMassA > 0) m->bodyA->position += n * (push * aShare);
                if (invMassB > 0) m->bodyB->position -= n * (push * bShare);
                diagSpikeProjCount++;
            }
        }
    }

    // --- 3d. Rest-state vertical velocity clamp (after heavy update & spike projection) ---
    // Purpose: if a dynamic body has multiple stable contacts and is slowly sinking with tiny penetration, damp its downward velocity.
    for (Rigid* b = bodies; b; b = b->next) {
        if (b->invMass <= 0) continue;
        if (b->groundedContacts >= 2) {
            if (b->linearVelocity.y < -0.05f) b->linearVelocity.y *= 0.5f; // strong damping when still moving downward
            else if (fabsf(b->linearVelocity.y) < 0.05f) b->linearVelocity.y = 0.0f; // snap to rest within threshold
        }
    }

    // --- 3d. Lateral Drift Damping (Stack COM stabilizer) ---
    // Compute average horizontal velocity of near-rest bodies and damp it to reduce slow sideways drift.
    {
        vec3 avgVel = {0,0,0};
        int count = 0;
        for (Rigid* b = bodies; b; b = b->next) {
            if (b->invMass <= 0) continue;
            // Consider bodies with small vertical motion and shallow corrections (rest candidates)
            if (fabsf(b->linearVelocity.y) < 0.6f) {
                avgVel += b->linearVelocity;
                count++;
            }
        }
        if (count > 0) {
            avgVel /= (float)count;
            // Only damp lateral components
            vec3 lateral = {avgVel.x, 0.0f, avgVel.z};
            float latLen = length(lateral);
            if (latLen > 0.0f) {
                float frameScale = (frameIndex < 240) ? 0.15f : 0.25f; // stronger later once stack mostly settled
                vec3 correction = lateral * frameScale;
                for (Rigid* b = bodies; b; b = b->next) {
                    if (b->invMass <= 0) continue;
                    if (fabsf(b->linearVelocity.y) < 0.6f) {
                        b->linearVelocity -= correction; // shift group toward zero lateral drift
                    }
                }
            }
        }
    }

    // --- 4. Main Iterative Solver Loop ---
skip_heuristics:
    int totalIterations = iterations + (postStabilize ? 1 : 0);
    for (int it = 0; it < totalIterations; ++it) {
        float currentAlpha = postStabilize ? (it < iterations ? 1.0f : 0.0f) : this->alpha;

        // --- 4a. Primal Update ---
        for (Rigid* body = bodies; body != 0; body = body->next) {
            debugPhase = "primal";
            debugInvokingBodyId = body->id;
            if (body->invMass <= 0) continue;

            mat3 M = mat3::diagonal(body->mass);
            // Use world-space inertia (not inverse) to properly weight angular corrections
            mat3 invI_world = body->getInvInertiaTensorWorld();
            mat3 I_world;
            if (!invert(invI_world, I_world)) {
                // Fallback: if inversion fails (shouldn't for valid bodies), approximate with identity scaled
                I_world = mat3::diagonal(1.0f / max(1e-6f, body->invMass));
            }

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
                // Read-only constraint recompute (no temporal mutation now) for current configuration
                force->computeConstraint(currentAlpha);
        if (force->isManifold()) diagCallsPrimal += ((Manifold*)force)->numContacts;
                for (int i = 0; i < force->getRowCount(); ++i) {
                    float oldPenaltyValue = force->penalty[i];
            // Skip rows with zero stiffness (friction rows are disabled in position solve)
            if (force->stiffness[i] == 0.0f) continue;

            float lambda_i = (force->stiffness[i] == FLT_MAX) ? force->lambda[i] : 0.0f;
            float f = clamp(force->penalty[i] * force->C[i] + lambda_i + force->motor[i], force->fmin[i], force->fmax[i]);

                    vec3 J_l, J_a;
                    force->computeDerivatives(J_l, J_a, body, i);
                    
                    rhs.l += J_l * f;
                    rhs.a += J_a * f;

                    // Accumulate normal row Jacobians only (penalty already excludes friction rows)
                    lhs.ll += outer_product(J_l, J_l) * force->penalty[i];
                    lhs.aa += outer_product(J_a, J_a) * force->penalty[i];
                    // Penalty smoothing: prevent sudden large drops (>30% in a single iteration)
                    if (force->penalty[i] < oldPenaltyValue * 0.70f) {
                        force->penalty[i] = oldPenaltyValue * 0.70f;
                    }
                }
            }

            // Solve and apply update
            vec6 dx = solve6x6(lhs, rhs);

            // Apply a fractional positional correction to avoid over-correcting in a single frame.
            // This mitigates jitter and reduces aggressive constraint energy build-up.
            float CORRECTION_FRACTION = 0.30f;
            if (frameIndex < 240) {
                CORRECTION_FRACTION = 0.30f + 0.12f * (frameIndex / 240.0f); // 0.30 -> 0.42
            } else {
                CORRECTION_FRACTION = 0.42f;
            }
            dx.l *= CORRECTION_FRACTION;
            dx.a *= CORRECTION_FRACTION;

            // Clamp linear & angular correction magnitudes per-iteration
            const float MAX_LINEAR_CORRECTION = 0.15f;   // meters per iteration (lower for safety)
            const float MAX_ANGULAR_CORRECTION = 0.25f;  // radians per iteration
            float linLen = length(dx.l);
            if (linLen > MAX_LINEAR_CORRECTION && linLen > 0) dx.l *= (MAX_LINEAR_CORRECTION / linLen);
            float angLen = length(dx.a);
            if (angLen > MAX_ANGULAR_CORRECTION && angLen > 0) dx.a *= (MAX_ANGULAR_CORRECTION / angLen);

            // Enforce per-frame cumulative caps to stop teleports if many iterations saturate
            const float FRAME_LINEAR_CAP = 0.4f;  // total positional change allowed per frame
            const float FRAME_ANGULAR_CAP = 0.6f; // total angular change allowed per frame
            float potentialLinearAccum = body->frameLinearCorrectionAccum + length(dx.l);
            if (potentialLinearAccum > FRAME_LINEAR_CAP && potentialLinearAccum > 0.0f) {
                float remain = FRAME_LINEAR_CAP - body->frameLinearCorrectionAccum;
                if (remain <= 0.0f) dx.l = vec3(0,0,0);
                else dx.l *= (remain / (potentialLinearAccum - body->frameLinearCorrectionAccum));
                body->frameLinearCorrectionAccum = FRAME_LINEAR_CAP;
            } else {
                body->frameLinearCorrectionAccum = potentialLinearAccum;
            }
            float potentialAngularAccum = body->frameAngularCorrectionAccum + length(dx.a);
            if (potentialAngularAccum > FRAME_ANGULAR_CAP && potentialAngularAccum > 0.0f) {
                float remainA = FRAME_ANGULAR_CAP - body->frameAngularCorrectionAccum;
                if (remainA <= 0.0f) dx.a = vec3(0,0,0);
                else dx.a *= (remainA / (potentialAngularAccum - body->frameAngularCorrectionAccum));
                body->frameAngularCorrectionAccum = FRAME_ANGULAR_CAP;
            } else {
                body->frameAngularCorrectionAccum = potentialAngularAccum;
            }

            // APPLY CORRECTIONS (SIGN FIX)
            // Previous code subtracted the solved displacement which drove bodies further INTO penetration.
            // We now add the correction so positive normal forces separate bodies.
            body->position += dx.l;
            quat dq(dx.a.x, dx.a.y, dx.a.z, 0);
            body->orientation = normalize(body->orientation + (dq * body->orientation) * 0.5f);
        }

        // --- 4b. Dual Update ---
        if (it < iterations) {
            debugPhase = "dual";
            for (Force* force = forces; force != 0; force = force->next) {
            // Reset per-frame jitter accumulators
            diagJitterVelSqSum = 0.0;
            diagJitterVelDeltaSqSum = 0.0;
            diagJitterVertSqSum = 0.0;
            diagJitterBodyCount = 0;
            diagJitterContactNormalVelSqSum = 0.0;
            diagJitterContactCount = 0;
            diagRestingNormalDampApplications = 0;
                force->computeConstraint(currentAlpha);
                if (force->isManifold()) diagCallsDual += ((Manifold*)force)->numContacts;
                for (int i = 0; i < force->getRowCount(); ++i) {
                    if (force->stiffness[i] != FLT_MAX) continue;
                    float lambda_i = clamp(force->penalty[i] * force->C[i] + force->lambda[i], force->fmin[i], force->fmax[i]);
                    force->lambda[i] = lambda_i;
                    
                    // Improved penalty parameter strategy
                    if (force->lambda[i] > force->fmin[i] && force->lambda[i] < force->fmax[i]) {
                        // Active constraint - increase penalty based on constraint violation and convergence
                        float constraint_violation = fabsf(force->C[i]);

                        // Base proposed increment (scaled down baseline)
                        float penalty_increment = beta * constraint_violation * 0.5f;

                        // Support boost: faster growth for near-vertical support normals to prevent slow stack collapse
                        if (force->isManifold()) {
                            Manifold* mGrow = (Manifold*)force;
                            int cidx = i / 3;
                            if ((i % 3)==0 && cidx < mGrow->numContacts) {
                                vec3 n = mGrow->contacts[cidx].normal;
                                if (n.y > 0.6f) {
                                    penalty_increment *= 1.8f; // base vertical support boost
                                    // Additional early boost if either body in the pair is bottom support layer
                                    bool bottomPair = (mGrow->bodyA && mGrow->bodyA->isBottomSupport) || (mGrow->bodyB && mGrow->bodyB->isBottomSupport);
                                    if (bottomPair && frameIndex < 240) {
                                        float t = 1.0f - (frameIndex / 240.0f);
                                        penalty_increment *= (1.5f + 1.0f * t);
                                    }
                                    // Support-specific penalty floor
                                    if (force->penalty[i] < 120.0f) force->penalty[i] = 120.0f;
                                }
                            }
                        }

                        // Hard clamp on absolute violation to avoid runaway when C is large
                        if (constraint_violation > 0.05f) {
                            penalty_increment *= 0.05f / constraint_violation; // scale down
                        }

                        // Relative growth clamp: no more than 25% per iteration
                        float maxRelativeIncrement = force->penalty[i] * 0.25f;
                        // For vertical supports allow higher relative growth (up to 60%) until moderately stiff
                        if (force->isManifold() && force->penalty[i] < 2e4f) {
                            Manifold* mGrow2 = (Manifold*)force;
                            int cidx2 = i / 3;
                            if ((i % 3)==0 && cidx2 < mGrow2->numContacts && mGrow2->contacts[cidx2].normal.y > 0.6f) {
                                maxRelativeIncrement = force->penalty[i] * 0.60f;
                            }
                        }
                        penalty_increment = min(penalty_increment, maxRelativeIncrement);

                        // Absolute per-iteration cap (safety)
                        const float ABS_PENALTY_INCREMENT_MAX = 1e4f; // tighter absolute cap with lower PENALTY_MAX
                        penalty_increment = min(penalty_increment, ABS_PENALTY_INCREMENT_MAX);

                        // Skip growth if contact already essentially at bias depth (prevents oscillatory stiffening)
                        if (force->isManifold() && (i % 3)==0) {
                            Manifold* mBias = (Manifold*)force;
                            int bidx = i / 3;
                            if (bidx < mBias->numContacts) {
                                float eff = mBias->contacts[bidx].effectivePenetration;
                                if (eff > 0.0f && eff < 0.0032f) { // near bias window -- allow small growth instead of zero
                                    penalty_increment *= 0.2f; // gentle growth
                                }
                            }
                        }
                        // Skip growth if already very stiff
                        if (force->penalty[i] > 5e4f) penalty_increment = 0.0f; // freeze growth earlier (half of new max)
                        // Ensure minimum moderate stiffness for sustained supports
                        if (force->isManifold() && (i % 3)==0) {
                            Manifold* mSup = (Manifold*)force; int cc = i/3; if (cc < mSup->numContacts && mSup->contacts[cc].normal.y > 0.6f) {
                                if (force->penalty[i] < 120.0f) force->penalty[i] = 120.0f;
                            }
                        }
                        // Adaptive relaxation: if manifold contact stable for long, soften instead of grow
                        bool requestedRelax = false;
                        if (force->isManifold()) {
                            Manifold* mrel = (Manifold*)force;
                            int contactIdx = i / 3;
                            if (contactIdx < mrel->numContacts && (i % 3) == 0) {
                                if (mrel->contacts[contactIdx].relaxPenalty) requestedRelax = true;
                            }
                        }
                        if (requestedRelax) {
                            float target = PENALTY_MIN * 4.0f; // don't relax below moderate stiffness
                            force->penalty[i] = std::max(target, force->penalty[i] * 0.95f);
                        } else {
                            force->penalty[i] = min(force->penalty[i] + penalty_increment, PENALTY_MAX);
                        }
                    } else {
                        // Inactive constraint - decay penalty to prevent over-stiffening
                        force->penalty[i] = max(force->penalty[i] * 0.95f, PENALTY_MIN);
                    }
                }
            }
        }
    }

    // --- 4c. Post-iteration anti-creep velocity adjustment ---
    // Cancel residual slow downward drift only for shallow, stable contacts and moderate relative sinking speeds.
    static int antiCreepCooldown = 0;
    if (antiCreepCooldown > 0) antiCreepCooldown--;
    int antiCreepApplications = 0;
    for (Force* force = forces; force; force = force->next) {
        if (!force->isManifold()) continue;
        Manifold* m = (Manifold*)force;
        for (int ci = 0; ci < m->numContacts; ++ci) {
            Manifold::Contact &c = m->contacts[ci];
            if (c.stableFrames < 120) continue; // wait longer before acting
            float eff = c.effectivePenetration;
            if (eff <= 0.0f || eff > 0.0025f) continue; // only very shallow supports
            float prev = c.prevEffectivePen;
            float dp = eff - prev; // change this frame
            if (dp < -2e-5f) continue; // only if not already getting shallower (avoid mid-phase push)
            Rigid* A = m->bodyA; Rigid* B = m->bodyB;
            vec3 relV = (A?A->linearVelocity:vec3()) - (B?B->linearVelocity:vec3());
            float relN = dot(relV, c.normal);
            if (relN < -0.12f && relN > -0.8f && antiCreepCooldown==0) { // moderate downward drift
                float cancel = fminf(fabsf(relN)*0.35f, 0.08f);
                if (A && A->invMass > 0) A->linearVelocity -= c.normal * (cancel * 0.5f);
                if (B && B->invMass > 0) B->linearVelocity += c.normal * (cancel * 0.5f);
                antiCreepApplications++;
            }
        }
    }
    if (antiCreepApplications>0) antiCreepCooldown = 5; // brief cooldown to prevent repeated trimming bursts

    // (Experimental contact pairwise projection disabled for now; relying on coupled 6x6 body solve.)

    // --- 5. Update Velocities (decoupled from positional corrections) ---
    for (Rigid* body = bodies; body != 0; body = body->next) {
        body->prevLinearVelocity = body->linearVelocity;
        body->prevAngularVelocity = body->angularVelocity;
        if (body->invMass <= 0) continue;

    // Base predicted velocity (ballistic). We intentionally DO NOT add a fraction of the
    // positional correction as velocity. Feeding positional correction into velocity was
    // injecting artificial kinetic energy (especially upward for stacking) because large
    // separation corrections were interpreted as physical motion. Contact impulses below
    // will establish proper resting (v_n -> 0) without this shortcut.
    body->linearVelocity = body->preSolveLinearVelocity + gravity * dt;

    // Angular: apply only a very small portion of orientation change to avoid sudden spins
    // yet still provide some damping of large corrective rotations.
    quat q_delta = body->orientation * conjugate(body->inertialOrientation);
    if (q_delta.w < 0) q_delta = quat(-q_delta.x, -q_delta.y, -q_delta.z, -q_delta.w);
    vec3 angApprox(q_delta.x, q_delta.y, q_delta.z);
    vec3 w_corr = (2.0f / dt) * angApprox;
    const float ANG_CORR_BLEND = 0.05f; // much smaller than previous 0.2 to curb energy gain
    body->angularVelocity = body->preSolveAngularVelocity + w_corr * ANG_CORR_BLEND;

        // Damping
        body->linearVelocity *= 0.995f;
        body->angularVelocity *= 0.99f;
    }

    // Small normal velocity clamp to suppress post-correction micro-bounce
    // --- Support Velocity Clamp (early stacking aid) ---
    // If a shallow contact exists (small effective penetration) we cap additional downward
    // approach speed so gravity does not accumulate large closing velocities before
    // positional solver can propagate support. This helps tall stacks settle earlier.
    {
        const float SUPPORT_PENETRATION_THRESHOLD = 0.005f; // 5mm
        const float MAX_DOWNWARD_APPROACH = -0.05f; // m/s allowed when shallowly in contact
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            for (int i = 0; i < m->numContacts; ++i) {
                const Manifold::Contact& c = m->contacts[i];
                if (c.effectivePenetration <= 0.0f || c.effectivePenetration > SUPPORT_PENETRATION_THRESHOLD) continue; // only very shallow
                vec3 n = c.normal; // points B -> A
                // Prefer contacts that oppose gravity (upward normals) to avoid clamping side contacts
                if (n.y < 0.5f) continue;
                vec3 rA = rotate(m->bodyA->orientation, c.rA);
                vec3 rB = rotate(m->bodyB->orientation, c.rB);
                vec3 vA = m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, rA);
                vec3 vB = m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, rB);
                vec3 v_rel = vA - vB;
                float v_n = dot(v_rel, n); // negative => approaching
                if (v_n >= MAX_DOWNWARD_APPROACH) continue; // already slow enough or separating
                float clampDelta = MAX_DOWNWARD_APPROACH - v_n; // positive amount to push toward limit
                // Distribute along normal inversely by mass (ignore rotation for simplicity)
                float invMassSum = m->bodyA->invMass + m->bodyB->invMass;
                if (invMassSum <= 0) continue;
                float ratioA = (m->bodyA->invMass > 0) ? (m->bodyA->invMass / invMassSum) : 0.0f;
                float ratioB = (m->bodyB->invMass > 0) ? (m->bodyB->invMass / invMassSum) : 0.0f;
                if (m->bodyA->invMass > 0) m->bodyA->linearVelocity += n * (clampDelta * ratioA);
                if (m->bodyB->invMass > 0) m->bodyB->linearVelocity -= n * (clampDelta * ratioB);
            }
        }
    }

    // --- Shallow Contact Normal Velocity Damping ---
    // For speculative / near-threshold contacts (very small effective penetration), scale down
    // any remaining approaching normal velocity to reduce oscillatory build-up before real
    // penetration develops. This is gentler than a hard clamp and only acts in the approach direction.
    {
        const float SHALLOW_DAMP_THRESHOLD = 0.005f; // <= 5mm effective penetration
        const float APPROACH_DAMP_FACTOR = 0.5f;     // retain only 50% of remaining approach
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            for (int i = 0; i < m->numContacts; ++i) {
                const Manifold::Contact& c = m->contacts[i];
                if (c.effectivePenetration <= 0.0f || c.effectivePenetration > SHALLOW_DAMP_THRESHOLD) continue;
                vec3 n = c.normal;
                if (n.y < 0.5f) continue; // restrict to upward-ish support normals
                vec3 rA = rotate(m->bodyA->orientation, c.rA);
                vec3 rB = rotate(m->bodyB->orientation, c.rB);
                vec3 vA = m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, rA);
                vec3 vB = m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, rB);
                vec3 v_rel = vA - vB;
                float v_n = dot(v_rel, n);
                if (v_n >= 0.0f) continue; // only damp approaching
                float target_v_n = v_n * APPROACH_DAMP_FACTOR; // less negative (closer to zero)
                float delta = target_v_n - v_n; // positive
                float invMassSum = m->bodyA->invMass + m->bodyB->invMass;
                if (invMassSum <= 0) continue;
                float ratioA = (m->bodyA->invMass > 0) ? (m->bodyA->invMass / invMassSum) : 0.0f;
                float ratioB = (m->bodyB->invMass > 0) ? (m->bodyB->invMass / invMassSum) : 0.0f;
                if (m->bodyA->invMass > 0) m->bodyA->linearVelocity += n * (delta * ratioA);
                if (m->bodyB->invMass > 0) m->bodyB->linearVelocity -= n * (delta * ratioB);
            }
        }
    }

    // --- Resting Angular Damping (variance-aware) ---
    // When multiple upward-facing contacts exist with small effective penetration average,
    // apply gentle angular damping to reduce residual rocking without affecting active motion.
    {
        int supportContacts = 0;
        float sumEff = 0.0f;
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            for (int i = 0; i < m->numContacts; ++i) {
                const Manifold::Contact& c = m->contacts[i];
                if (c.effectivePenetration <= 0) continue;
                if (c.normal.y < 0.5f) continue; // only upward-ish
                supportContacts++;
                sumEff += c.effectivePenetration;
            }
        }
        if (supportContacts >= 6) { // enough contacts across stack
            float avgEff = sumEff / supportContacts;
            // Compute variance among upward support effective penetrations
            float varSum = 0.0f; int varCount = 0;
            for (Force* f2 = forces; f2; f2 = f2->next) {
                if (!f2->isManifold()) continue; Manifold* m2 = (Manifold*)f2;
                for (int i=0;i<m2->numContacts;++i) { const Manifold::Contact& c = m2->contacts[i]; if (c.effectivePenetration<=0||c.normal.y<0.5f) continue; varSum += (c.effectivePenetration-avgEff)*(c.effectivePenetration-avgEff); varCount++; }
            }
            float variance = (varCount>0) ? (varSum/varCount) : 0.0f;
            if (avgEff < 0.006f) {
                float damp = 0.85f; // base
                if (variance > 1e-6f) damp *= 0.80f; // stronger if uneven support
                for (Rigid* body = bodies; body; body = body->next) { if (body->invMass <= 0) continue; body->angularVelocity *= damp; }
            }
        }
    }

    // --- Creep Guard Bias Boost (bottom supports) ---
    if (frameIndex % 90 == 0) {
        float avgTopVy = 0.0f; int topCount = 0; std::vector<std::pair<float,Rigid*>> bodyHeights;
        for (Rigid* b = bodies; b; b = b->next) { if (b->invMass<=0) continue; bodyHeights.emplace_back(b->position.y,b); }
        if (!bodyHeights.empty()) {
            std::sort(bodyHeights.begin(), bodyHeights.end(), [](auto&a,auto&b){return a.first<b.first;});
            size_t start = (size_t)(bodyHeights.size()*0.75);
            for (size_t k=start;k<bodyHeights.size();++k){ avgTopVy += bodyHeights[k].second->linearVelocity.y; topCount++; }
            if (topCount>0) avgTopVy/=topCount;
        }
        if (avgTopVy < -0.15f) { // sinking trend
            for (Force* force = forces; force; force = force->next) {
                if (!force->isManifold()) continue; Manifold* m = (Manifold*)force;
                for (int i=0;i<m->numContacts;++i) {
                    Manifold::Contact& c = m->contacts[i];
                    if (c.normal.y > 0.9f && c.effectivePenetration>0.0f && c.effectivePenetration < 0.0035f) {
                        c.prevEffectivePen = std::max(c.prevEffectivePen, 0.0035f);
                    }
                }
            }
        }
    }

    for (Force* force = forces; force; force = force->next) {
        if (!force->isManifold()) continue;
        Manifold* m = (Manifold*)force;
        for (int i = 0; i < m->numContacts; ++i) {
            const Manifold::Contact& c = m->contacts[i];
            vec3 rA = rotate(m->bodyA->orientation, c.rA);
            vec3 rB = rotate(m->bodyB->orientation, c.rB);
            vec3 v_rel = (m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, rA)) -
                         (m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, rB));
            float v_n = dot(v_rel, c.normal);
            // Legacy small-separation clamp replaced by controlled resting damping below a threshold
            if (fabsf(v_n) < restingNormalDampThreshold) {
                // Remove a fraction of relative normal velocity to kill micro-oscillations
                float remove = v_n * restingNormalDampFactor; // signed (reduces toward zero)
                float invMassSum = m->bodyA->invMass + m->bodyB->invMass;
                if (invMassSum > 0.0f && fabsf(remove) > 1e-6f) {
                    float aRatio = (m->bodyA->invMass > 0) ? (m->bodyA->invMass / invMassSum) : 0.0f;
                    float bRatio = (m->bodyB->invMass > 0) ? (m->bodyB->invMass / invMassSum) : 0.0f;
                    if (m->bodyA->invMass > 0) m->bodyA->linearVelocity -= c.normal * (remove * aRatio);
                    if (m->bodyB->invMass > 0) m->bodyB->linearVelocity += c.normal * (remove * bRatio);
                    diagRestingNormalDampApplications++;
                }
            }
            // Spike approach damping: when a contact flagged as spike, limit relative approach speed aggressively
            if (c.spikeThisFrame) {
                vec3 n = c.normal;
                if (n.y > 0.3f) { // mainly support-like
                    float approach = dot(v_rel, n); // negative = approaching
                    if (approach < -0.2f) {
                        float target = -0.2f;
                        float delta = target - approach; // positive
                        float invMassSum = m->bodyA->invMass + m->bodyB->invMass;
                        if (invMassSum > 0.0f) {
                            float aRatio = (m->bodyA->invMass > 0) ? (m->bodyA->invMass / invMassSum) : 0.0f;
                            float bRatio = (m->bodyB->invMass > 0) ? (m->bodyB->invMass / invMassSum) : 0.0f;
                            if (m->bodyA->invMass > 0) m->bodyA->linearVelocity += n * (delta * aRatio);
                            if (m->bodyB->invMass > 0) m->bodyB->linearVelocity -= n * (delta * bRatio);
                        }
                    }
                }
            }
        }
    }

#if ENABLE_SOLVER_DIAGNOSTICS
    // --- Diagnostics (Constraint Violation & Penalty Stats) ---
    if (frameIndex % SOLVER_DIAG_PERIOD == 0 || frameIndex==600 || frameIndex==780) {
    float maxC = 0.0f;
    float sumPenalty = 0.0f;
    int normalCount = 0;
    int frozenCount = 0;
    float maxRawPen = 0.0f;
    float sumEffectivePen = 0.0f;
    float sumRawPen = 0.0f;
    int spikeContacts = 0;
    int clampedCount = 0;
    // Penetration histogram buckets (raw) for distribution insight
    int penBuckets[6] = {0,0,0,0,0,0}; // 0:[0-1mm) 1:[1-2mm) 2:[2-4mm) 3:[4-8mm) 4:[8-20mm) 5:>=20mm
        const float PENALTY_FREEZE_THRESHOLD = 5e4f; // matches growth freeze threshold

        // Recompute current constraint violations for accurate diagnostics
        for (Force* force = forces; force != 0; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            debugPhase = "diagnostics";
            debugInvokingBodyId = -1;
            force->computeConstraint(alpha); // read-only refresh (temporal already done in projection pass)
            if (force->isManifold()) diagCallsDiagnostics += ((Manifold*)force)->numContacts;
            for (int ci = 0; ci < m->numContacts; ++ci) {
                const Manifold::Contact& c = m->contacts[ci];
                float rawPen = c.penetration;
                if (rawPen > maxRawPen) maxRawPen = rawPen;
                sumRawPen += rawPen;
                if (c.spikeThisFrame) spikeContacts++;
                sumEffectivePen += c.effectivePenetration;
                if (c.penetrationClamped) clampedCount++;
                // Histogram classify (convert meters to mm for bucket logic)
                float mm = rawPen * 1000.0f;
                if (mm < 1.0f) penBuckets[0]++; else if (mm < 2.0f) penBuckets[1]++; else if (mm < 4.0f) penBuckets[2]++; else if (mm < 8.0f) penBuckets[3]++; else if (mm < 20.0f) penBuckets[4]++; else penBuckets[5]++;
                // normal row index
                int r = ci*3 + 0;
                float Cval = force->C[r];
                if (Cval > maxC) maxC = Cval;
                sumPenalty += force->penalty[r];
                normalCount++;
                if (force->penalty[r] >= PENALTY_FREEZE_THRESHOLD) frozenCount++;
            }
        }

        float avgPenalty = (normalCount > 0) ? (sumPenalty / normalCount) : 0.0f;
        float totalLinearCorrection = 0.0f;
        for (Rigid* body = bodies; body != 0; body = body->next) {
            totalLinearCorrection += body->frameLinearCorrectionAccum;
        }
    float avgEffectivePen = (normalCount > 0) ? (sumEffectivePen / normalCount) : 0.0f;
    float avgRawPen = (normalCount > 0) ? (sumRawPen / normalCount) : 0.0f;
        // Compute stack COM & potential energy (relative to lowest body y) + accumulate jitter metrics
        int bodyCount = 0;
        vec3 com = {0,0,0};
        float totalMass = 0.0f;
        float minY = 1e9f;
        for (Rigid* b = bodies; b; b = b->next) {
            if (b->invMass <= 0) continue;
            float m = b->mass;
            totalMass += m;
            com += b->position * m;
            if (b->position.y < minY) minY = b->position.y;
            bodyCount++;
            // Velocity jitter (per-body): squared magnitude and frame-to-frame delta
            diagJitterVelSqSum += dot(b->linearVelocity, b->linearVelocity);
            vec3 dv = b->linearVelocity - b->prevLinearVelocity;
            diagJitterVelDeltaSqSum += dot(dv,dv);
            diagJitterVertSqSum += b->linearVelocity.y * b->linearVelocity.y;
            diagJitterBodyCount++;
        }
        // Accumulate per-contact normal relative velocity jitter
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            for (int ci=0; ci < m->numContacts; ++ci) {
                Manifold::Contact &c = m->contacts[ci];
                // relative velocity along normal (reuse earlier compute? we recompute quickly here)
                vec3 v_rel = (m->bodyA->linearVelocity - m->bodyB->linearVelocity);
                float nrel = dot(v_rel, c.normal);
                diagJitterContactNormalVelSqSum += double(nrel)*double(nrel);
                diagJitterContactCount++;
            }
        }
        if (totalMass > 0) com /= totalMass;
        // COM jitter (difference from previous COM) – reuse prevCOM storage
        double comJitterSq = 0.0;
        if (hasPrevCOM) {
            vec3 dcom = com - prevCOM;
            comJitterSq = dot(dcom,dcom);
        }
        prevCOM = com; hasPrevCOM = true;
        float potential = 0.0f;
        for (Rigid* b = bodies; b; b = b->next) {
            if (b->invMass <= 0) continue;
            potential += b->mass * (b->position.y - minY) * fabsf(gravity.y);
        }
        // Average vertical velocity of top 25% bodies (by y)
        std::vector<float> ys;
        std::vector<float> vy;
        for (Rigid* b = bodies; b; b = b->next) {
            if (b->invMass <= 0) continue;
            ys.push_back(b->position.y);
            vy.push_back(b->linearVelocity.y);
        }
        float avgTopVy = 0.0f;
        if (!ys.empty()) {
            // Find cutoff
            std::vector<size_t> idx(ys.size());
            for (size_t i=0;i<idx.size();++i) idx[i]=i;
            std::sort(idx.begin(), idx.end(), [&](size_t a,size_t b){return ys[a]<ys[b];});
            size_t start = (size_t)((double)idx.size()*0.75);
            size_t count = 0;
            for (size_t k=start;k<idx.size();++k){ avgTopVy += vy[idx[k]]; ++count; }
            if (count>0) avgTopVy/=count;
        }
    // Rolling maxPen over last 30 frames (simple decay approach)
    static float rollingMaxPen = 0.0f;
    rollingMaxPen *= 0.90f; // decay
    if (maxRawPen > rollingMaxPen) rollingMaxPen = maxRawPen;
    // Compute average lateral velocity (near-rest bodies) for diagnostics
    float avgLatVel = 0.0f; int latCount=0; for (Rigid* bTmp = bodies; bTmp; bTmp = bTmp->next) { if (bTmp->invMass>0 && fabsf(bTmp->linearVelocity.y)<0.8f) { vec3 lv=bTmp->linearVelocity; lv.y=0; avgLatVel += length(lv); latCount++; }} if (latCount>0) avgLatVel/=latCount;
    // Support load stats: average supportLoad top vs bottom quartile
    std::vector<Rigid*> dyn; dyn.reserve(bodyCount);
    for (Rigid* b = bodies; b; b = b->next) if (b->invMass>0) dyn.push_back(b);
    float bottomLoadAvg=0, topLoadAvg=0; int bottomCount=0, topCount=0;
    if (!dyn.empty()) {
        std::sort(dyn.begin(), dyn.end(), [](Rigid* a, Rigid* b){return a->position.y < b->position.y;});
        size_t slice = std::max<size_t>(1, dyn.size()/4);
        for (size_t i=0;i<slice;++i){ bottomLoadAvg += dyn[i]->supportLoad; bottomCount++; }
        for (size_t i=dyn.size()-slice;i<dyn.size();++i){ topLoadAvg += dyn[i]->supportLoad; topCount++; }
        if (bottomCount) bottomLoadAvg/=bottomCount;
        if (topCount) topLoadAvg/=topCount;
    }
    float totalMassDyn=0.0f, maxMassDyn=0.0f; for (Rigid* b = bodies; b; b=b->next) if (b->invMass>0){ totalMassDyn+=b->mass; if(b->mass>maxMassDyn) maxMassDyn=b->mass; }
        float avgMassDyn = bodyCount>0 ? totalMassDyn/bodyCount : 0.0f;
        // --- CSV Logging (lightweight) ---
        // Emit one row per diagnostics sample period with core metrics for offline comparison.
        if (csvLogEnabled && csvLogFile) {
            // Columns (jitter extended): frame,maxPen,avgEffPen,avgRawPen,avgPenalty,manifolds,contacts,jitRMSVel,jitRMSVelDelta,jitRMSVert,jitCOM,jitRMSContactN,topVy,totalLinCorr,manifoldsDropped,manifoldsResurrected
            // Use maxRawPen as maxPen (pre any smoothing), avgEffectivePen (post clamp), avgRawPen (pre clamp)
            double rmsVel = (diagJitterBodyCount>0)? sqrt(diagJitterVelSqSum/diagJitterBodyCount):0.0;
            double rmsVelDelta = (diagJitterBodyCount>0)? sqrt(diagJitterVelDeltaSqSum/diagJitterBodyCount):0.0;
            double rmsVert = (diagJitterBodyCount>0)? sqrt(diagJitterVertSqSum/diagJitterBodyCount):0.0;
            double comJ = sqrt(comJitterSq);
            double rmsContactN = (diagJitterContactCount>0)? sqrt(diagJitterContactNormalVelSqSum/diagJitterContactCount):0.0;
        fprintf(csvLogFile, "%d,%.6f,%.6f,%.6f,%.1f,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.3f,%d,%d\n",
                    frameIndex,
                    maxRawPen,
                    avgEffectivePen,
                    avgRawPen,
                    avgPenalty,
                    diagManifoldCount,
            diagContactCount,
                    rmsVel,
                    rmsVelDelta,
                    rmsVert,
                    comJ,
                    rmsContactN,
                    avgTopVy,
            totalLinearCorrection,
            diagManifoldsDropped,
            diagManifoldsResurrected);
            fflush(csvLogFile);
        }
    printf("[Diag] frame=%d maxC=%.5f maxPen=%.4f roll30=%.4f avgEffPen=%.4f avgRawPen=%.4f spikes=%d clamps=%d/%d axisClamps=%d spikeProj=%d avgPenalty=%.1f normals=%d frozen=%d totalLinCorr=%.3f latV=%.4f penHist[%d,%d,%d,%d,%d,%d] COM(%.3f,%.3f,%.3f) pot=%.3f topVy=%.3f calls=%lld heavy=%lld (proj=%lld prim=%lld dual=%lld diag=%lld) manifolds=%d dropped=%d resurrect=%d antiCreep=%d mass(avg=%.2f max=%.2f n=%d) load(bottom=%.4f top=%.4f)\n",
           frameIndex, maxC, maxRawPen, rollingMaxPen, avgEffectivePen, avgRawPen, spikeContacts, clampedCount, normalCount, diagAxisClampCount, diagSpikeProjCount, avgPenalty, normalCount, frozenCount, totalLinearCorrection, avgLatVel,
               penBuckets[0],penBuckets[1],penBuckets[2],penBuckets[3],penBuckets[4],penBuckets[5],
               com.x, com.y, com.z, potential, avgTopVy,
               (long long)diagConstraintComputeCalls, (long long)diagConstraintHeavyUpdateCalls, (long long)diagCallsProjection, (long long)diagCallsPrimal, (long long)diagCallsDual, (long long)diagCallsDiagnostics,
               diagManifoldCount, diagManifoldsDropped, diagManifoldsResurrected, antiCreepApplications,
               avgMassDyn, maxMassDyn, bodyCount, bottomLoadAvg, topLoadAvg);
            
    }
#endif

    // --- 6. Velocity Solve for Restitution (e=0) ---
    // --- FIX ---
    // The previous iterative velocity solve (vel_iterations = 10) was incorrectly
    // applying a full corrective impulse in every iteration without accumulating the
    // results. This created a positive feedback loop that injected massive amounts
    // of energy, causing bouncing and accelerating sliding. A single, non-iterative
    // pass is the correct approach for this type of post-correction step.
    const int vel_iterations = 1;
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

                vec3 impulse = j * n;

                if (m->bodyA->invMass > 0) {
                    m->bodyA->linearVelocity += m->bodyA->invMass * impulse;
                    m->bodyA->angularVelocity += m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, impulse);
                }

                if (m->bodyB->invMass > 0) {
                    m->bodyB->linearVelocity -= m->bodyB->invMass * impulse;
                    m->bodyB->angularVelocity -= m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, impulse);
                }

        // Friction (static + dynamic)
        v_rel = (m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, world_rA)) -
            (m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, world_rB));
        vec3 tangent1, tangent2;
        if (fabsf(n.x) > 0.9f) tangent1 = normalize(cross(n, vec3(0,1,0))); else tangent1 = normalize(cross(n, vec3(1,0,0))); tangent2 = normalize(cross(n, tangent1));
        vec3 Ia_t1 = m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, tangent1); float qa_t1 = dot(cross(Ia_t1, world_rA), tangent1); vec3 Ib_t1 = m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, tangent1); float qb_t1 = dot(cross(Ib_t1, world_rB), tangent1); float em_t1 = m->bodyA->invMass + m->bodyB->invMass + qa_t1 + qb_t1;
        vec3 Ia_t2 = m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, tangent2); float qa_t2 = dot(cross(Ia_t2, world_rA), tangent2); vec3 Ib_t2 = m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, tangent2); float qb_t2 = dot(cross(Ib_t2, world_rB), tangent2); float em_t2 = m->bodyA->invMass + m->bodyB->invMass + qa_t2 + qb_t2;
        float frictionBoost = 1.0f; if (frameIndex < 300) { float early = (frameIndex < 180)?0.0f:(float)(frameIndex-180)/120.0f; frictionBoost = 1.0f + (1.4f-1.0f)*(1.0f-early); }
        float mu = m->combinedFriction * frictionBoost; float vt1 = dot(v_rel, tangent1); float vt2 = dot(v_rel, tangent2); float vtMag = sqrtf(vt1*vt1+vt2*vt2);
        // Adaptive static friction threshold: rises slightly after early settling to better lock resting contacts
        float staticThresh = (frameIndex < 200 ? 0.05f : 0.08f);
        float jt1=0, jt2=0;
        if (vtMag < 1e-7f) { } else if (vtMag < staticThresh) { float jt_need_1 = (em_t1>0)? -vt1/em_t1:0.0f; float jt_need_2 = (em_t2>0)? -vt2/em_t2:0.0f; float needMag = sqrtf(jt_need_1*jt_need_1+jt_need_2*jt_need_2); float staticLimit = mu*1.2f*j; if (needMag <= staticLimit) { jt1=jt_need_1; jt2=jt_need_2; } else { float sc = staticLimit/(needMag+1e-9f); jt1=jt_need_1*sc; jt2=jt_need_2*sc; } }
        else { jt1 = (em_t1>0)? -vt1/em_t1:0.0f; jt2 = (em_t2>0)? -vt2/em_t2:0.0f; float dynLimit = mu*j; float mag = sqrtf(jt1*jt1+jt2*jt2); if (mag > dynLimit) { float sc = dynLimit/(mag+1e-9f); jt1*=sc; jt2*=sc; } }
        vec3 friction_impulse = tangent1*jt1 + tangent2*jt2; if (m->bodyA->invMass>0){ m->bodyA->linearVelocity += m->bodyA->invMass*friction_impulse; m->bodyA->angularVelocity += m->bodyA->getInvInertiaTensorWorld()*cross(world_rA, friction_impulse);} if (m->bodyB->invMass>0){ m->bodyB->linearVelocity -= m->bodyB->invMass*friction_impulse; m->bodyB->angularVelocity -= m->bodyB->getInvInertiaTensorWorld()*cross(world_rB, friction_impulse);}                
            }
        }
    }

    // --- 7A. Resting Contact Stabilization (micro-jitter suppression) ---
    // For shallow, near-rest contacts remove residual normal relative velocity and damp tiny lateral slip.
    {
        for (Force* force = forces; force; force = force->next) {
            if (!force->isManifold()) continue;
            Manifold* m = (Manifold*)force;
            for (int i = 0; i < m->numContacts; ++i) {
                Manifold::Contact& c = m->contacts[i];
                // Only act on modest penetrations (support band)
                if (c.effectivePenetration < 0.0025f || c.effectivePenetration > 0.012f) continue;
                vec3 n = c.normal;
                // World offsets
                vec3 world_rA = rotate(m->bodyA->orientation, c.rA);
                vec3 world_rB = rotate(m->bodyB->orientation, c.rB);
                vec3 v_rel = (m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, world_rA)) -
                             (m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, world_rB));
                float v_n = dot(v_rel, n);
                float abs_v_n = fabsf(v_n);
                // Skip energetic contacts
                if (abs_v_n > 0.35f) continue;
                // Effective mass (reuse from normal impulse logic)
                vec3 Ia_n = m->bodyA->getInvInertiaTensorWorld() * cross(world_rA, n);
                float qa = dot(cross(Ia_n, world_rA), n);
                vec3 Ib_n = m->bodyB->getInvInertiaTensorWorld() * cross(world_rB, n);
                float qb = dot(cross(Ib_n, world_rB), n);
                float em = m->bodyA->invMass + m->bodyB->invMass + qa + qb;
                if (em <= 0) continue;
                // Apply scaled corrective impulse to damp residual normal jitter
                float dampScale = 0.85f; // fraction of residual normal velocity to kill
                if (abs_v_n > 1e-4f) {
                    float j = -dampScale * v_n / em;
                    vec3 impulse = j * n;
                    if (m->bodyA->invMass>0){ m->bodyA->linearVelocity += m->bodyA->invMass*impulse; m->bodyA->angularVelocity += m->bodyA->getInvInertiaTensorWorld()*cross(world_rA, impulse);} 
                    if (m->bodyB->invMass>0){ m->bodyB->linearVelocity -= m->bodyB->invMass*impulse; m->bodyB->angularVelocity -= m->bodyB->getInvInertiaTensorWorld()*cross(world_rB, impulse);} 
                }
                // Tangential micro-slip damping
                vec3 v_rel2 = (m->bodyA->linearVelocity + cross(m->bodyA->angularVelocity, world_rA)) -
                              (m->bodyB->linearVelocity + cross(m->bodyB->angularVelocity, world_rB));
                vec3 vt = v_rel2 - n * dot(v_rel2, n);
                float vtLen = length(vt);
                if (vtLen > 1e-5f && vtLen < 0.06f) {
                    float reduce = 0.75f; // remove 75% of tiny drift
                    vec3 vtKill = -reduce * vt;
                    // Distribute as velocity change (approximate impulse)
                    if (m->bodyA->invMass>0) m->bodyA->linearVelocity += vtKill * 0.5f;
                    if (m->bodyB->invMass>0) m->bodyB->linearVelocity -= vtKill * 0.5f;
                }
            }
        }
    }

    // --- 7. Post-Velocity Base Angular Damping ---
    // Kill residual rocking at the base once linear motion small.
    {
        float heightMin = 1e9f, heightMax = -1e9f;
        for (Rigid* b = bodies; b; b = b->next) if (b->invMass>0) { if (b->position.y < heightMin) heightMin = b->position.y; if (b->position.y > heightMax) heightMax = b->position.y; }
        float heightSpan = heightMax - heightMin;
        if (heightSpan > 0) {
            for (Rigid* b = bodies; b; b = b->next) {
                if (b->invMass <= 0) continue;
                float rel = (b->position.y - heightMin) / heightSpan; // 0=base
                if (rel < 0.35f && length(b->linearVelocity) < 0.8f) {
                    float damp = 0.5f + 0.5f * (1.0f - rel / 0.35f); // stronger at very base
                    b->angularVelocity.x *= (1.0f - 0.75f * damp);
                    b->angularVelocity.z *= (1.0f - 0.75f * damp);
                    b->angularVelocity.y *= (1.0f - 0.4f * damp);
                }
            }
        }
    }

    // --- 8. Layer Alignment Damping ---
    // For vertically adjacent bodies with small relative vertical offset, damp horizontal relative velocity to reduce shear drift.
    {
        const float VERTICAL_NEIGHBOR_THRESH = 1.2f; // meters
        for (Rigid* A = bodies; A; A = A->next) {
            if (A->invMass <= 0) continue;
            for (Rigid* B = A->next; B; B = B->next) {
                if (B->invMass <= 0) continue;
                float dy = fabsf(B->position.y - A->position.y);
                if (dy > VERTICAL_NEIGHBOR_THRESH) continue;
                vec3 rel = B->linearVelocity - A->linearVelocity;
                vec3 lateral = {rel.x, 0.0f, rel.z};
                float latLen = length(lateral);
                if (latLen < 1e-6f) continue;
                if (latLen > 0.02f) { // only act if noticeable drift
                    float damp = 0.25f; // mild
                    vec3 corr = lateral * damp;
                    float invSum = A->invMass + B->invMass;
                    if (invSum <= 0) continue;
                    if (A->invMass > 0) A->linearVelocity += corr * (A->invMass / invSum);
                    if (B->invMass > 0) B->linearVelocity -= corr * (B->invMass / invSum);
                }
            }
        }
    }

    // --- 9. Gentle Lateral COM Recentering (drift correction) ---
    // After early settling, if the whole stack has drifted laterally but is otherwise calm, nudge it back toward initial COM.
    {
        static bool baseComCaptured = false;
        static vec3 baseCom = {0,0,0};
        if (!baseComCaptured) {
            // Capture initial COM of dynamic bodies (frame 0)
            float msum = 0.0f; vec3 acc = {0,0,0};
            for (Rigid* b = bodies; b; b = b->next) if (b->invMass>0) { acc += b->position * b->mass; msum += b->mass; }
            if (msum > 0) baseCom = acc / msum;
            baseComCaptured = true;
        }
        if (frameIndex > 220) {
            float msum = 0.0f; vec3 acc = {0,0,0}; vec3 velAcc = {0,0,0};
            for (Rigid* b = bodies; b; b = b->next) if (b->invMass>0) { msum += b->mass; acc += b->position * b->mass; velAcc += b->linearVelocity * b->mass; }
            if (msum > 0) {
                vec3 com = acc / msum;
                vec3 vcom = velAcc / msum;
                vec3 lateralOffset = {com.x - baseCom.x, 0.0f, com.z - baseCom.z};
                float offLen = length(lateralOffset);
                float lateralSpeed = length(vec3{vcom.x,0.0f,vcom.z});
                // Conditions: significant offset, calm lateral motion
                if (offLen > 0.05f && lateralSpeed < 0.25f) {
                    // Strength scales down with offset to avoid overshoot, and with frame dt
                    float strength = fminf(0.6f, offLen) * 0.35f; // base factor
                    vec3 correctionVel = (-strength) * lateralOffset; // direction back to base
                    for (Rigid* b = bodies; b; b = b->next) if (b->invMass>0) {
                        float w = b->mass / msum;
                        b->linearVelocity += correctionVel * w; // mass-weighted distribution
                    }
                }
            }
        }
    }

    // (Optional future: track base angular diagnostic) currently integrated by printing maxC & others.
}

void Solver::draw() {
    for (Rigid* body = bodies; body != 0; body = body->next) body->draw();
    for (Force* force = forces; force != 0; force = force->next) force->draw();
}