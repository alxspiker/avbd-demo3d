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

// Forward declaration of tracing helper defined in solver.cpp
bool solver_should_trace(const Solver* s);

// A small penetration tolerance to prevent jitter from solver over-correction.
const float PENETRATION_SLOP = 0.001f; // 1mm

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB)
    : Force(solver, bodyA, bodyB), numContacts(0)
{
    computeCallsThisFrame = 0;
    // The constructor is minimal, all work is done in initialize
    for (int i = 0; i < 8; ++i) {
        contacts[i].C0_n = 0.0f;
        contacts[i].C0_t = vec3{0, 0, 0};
    contacts[i].prevEffectivePen = 0.0f;
    contacts[i].stableFrames = 0;
    contacts[i].relaxPenalty = false;
    contacts[i].spikeThisFrame = false;
    contacts[i].prevRawPen = 0.0f;
    contacts[i].spikeCooldown = 0;
    contacts[i].depthMemory = 0.0f;
    contacts[i].memoryAge = 0;
    }
}

int Manifold::getRowCount() const {
    return numContacts * 3; // 1 Normal, 2 Tangent friction rows per contact
}

bool Manifold::initialize() {
    combinedFriction = sqrtf(bodyA->friction * bodyB->friction);

    // --- Warm Starting Cache ---
    Contact oldContacts[8];
    float oldLambda[24];
    int oldNumContacts = numContacts;
    for (int i = 0; i < oldNumContacts; ++i) oldContacts[i] = contacts[i];
    for (int i = 0; i < oldNumContacts * 3; ++i) oldLambda[i] = lambda[i];

    // --- Get New Contacts ---
    numContacts = Manifold::collide(bodyA, bodyB, contacts, false);
    if (numContacts == 0) {
        bool resurrected = false;
        // Persistence hysteresis: if previous contacts existed and had a depthMemory recently, keep a speculative one
        if (oldNumContacts > 0) {
            // Collect eligible old contacts
            struct Hist { float depth; int idx; } hist[4]; int hcount=0;
            for (int j=0;j<oldNumContacts;++j) {
                if (oldContacts[j].depthMemory > 0.0f && oldContacts[j].memoryAge < 90) {
                    hist[hcount++] = { oldContacts[j].depthMemory, j };
                }
            }
            if (hcount>0) {
                // sort by depth desc
                for (int a=0;a<hcount-1;++a) for (int b=a+1;b<hcount;++b) if (hist[b].depth > hist[a].depth) std::swap(hist[a], hist[b]);
                int kept = 0;
                for (int k=0;k<hcount && kept<2 && kept<4; ++k) {
                    contacts[kept] = oldContacts[hist[k].idx];
                    contacts[kept].penetration = contacts[kept].depthMemory; // shallow depth
                    kept++;
                }
                numContacts = kept;
                resurrected = true;
            }
        }
        if (!resurrected) return false;
        if (resurrected && bodyA && bodyA->solver) bodyA->solver->diagManifoldsResurrected++;
        resurrectedThisFrame = resurrected;
    } else {
        resurrectedThisFrame = false;
    }
    framesAlive++; // manifold survived another frame
    if (numContacts>0) framesSincePenetration = 0; else framesSincePenetration++;

    // Set stiffness to FLT_MAX for normal rows and 0 for friction rows
    // Enable a small positional friction stiffness after early settling window (reduces lateral creep without high iteration budget)
    float frictionPosStiff = 0.0f;
    if (solver) {
        int f = solver->frameIndex;
        if (f > 120) {
            float t = fminf(1.0f, (f - 120) / 80.0f); // ramp 120->200
            frictionPosStiff = 5.0f * t;
        }
    }
    for (int i = 0; i < numContacts; ++i) {
        stiffness[i*3 + 0] = FLT_MAX;          // normal (hard)
        stiffness[i*3 + 1] = frictionPosStiff; // tangent 1
        stiffness[i*3 + 2] = frictionPosStiff; // tangent 2
    }

    // --- Match New Contacts with Old for Warm Starting ---
    for (int i = 0; i < numContacts; ++i) {
        lambda[i*3 + 0] = lambda[i*3 + 1] = lambda[i*3 + 2] = 0.0f;
        penalty[i*3 + 0] = penalty[i*3 + 1] = penalty[i*3 + 2] = PENALTY_MIN;
        contacts[i].stick = false;
        contacts[i].relaxPenalty = false;

        for (int j = 0; j < oldNumContacts; ++j) {
            if (contacts[i].feature.value == oldContacts[j].feature.value) {
                lambda[i*3 + 0] = oldLambda[j*3 + 0];
                lambda[i*3 + 1] = oldLambda[j*3 + 1];
                lambda[i*3 + 2] = oldLambda[j*3 + 2];
                contacts[i].stick = oldContacts[j].stick;
                // Carry over persistence memory
                contacts[i].depthMemory = oldContacts[j].depthMemory;
                contacts[i].memoryAge = oldContacts[j].memoryAge;
                
                // For sticking contacts, reuse the exact previous contact points
                if(contacts[i].stick) {
                    contacts[i].rA = oldContacts[j].rA;
                    contacts[i].rB = oldContacts[j].rB;
                }
                break;
            }
        }

        // If this is a speculative or extremely shallow contact (tiny depth from broadphase proximity),
        // discard any inherited normal lambda to avoid over-shooting/jitter when alpha > 0.
        // This keeps early support formation gentle and reduces oscillation sensitivity to the alpha slider.
        const float SHALLOW_PEN_THRESHOLD = 0.0035f; // 3.5mm
    if (contacts[i].penetration > 0.0f && contacts[i].penetration <= SHALLOW_PEN_THRESHOLD) {
            lambda[i*3 + 0] = 0.0f; // force fresh solve accumulation
        }
    }

    // --- Pre-computation for Taylor Series Approximation (following 2D reference) ---
    for (int i = 0; i < numContacts; ++i) {
        vec3 world_rA = rotate(bodyA->orientation, contacts[i].rA);
        vec3 world_rB = rotate(bodyB->orientation, contacts[i].rB);
        
        vec3 pA = bodyA->position + world_rA;
        vec3 pB = bodyB->position + world_rB;
        vec3 normal = contacts[i].normal;
        
        // Precompute C0 - the constraint violation at the beginning of this frame
        // Following 2D: C0 = basis * (pA - pB) + collision_margin
        // Use the stored penetration depth from collision detection
        contacts[i].C0_n = -contacts[i].penetration + PENETRATION_SLOP; // Negative because penetration is positive when objects overlap
        
        // For tangent directions (friction) - compute relative velocity projected onto tangent
        vec3 vrel = (bodyA->linearVelocity + cross(bodyA->angularVelocity, world_rA)) -
                    (bodyB->linearVelocity + cross(bodyB->angularVelocity, world_rB));

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
    computeCallsThisFrame++;
    if (bodyA && bodyA->solver) bodyA->solver->diagConstraintComputeCalls++;
    for (int i = 0; i < numContacts; ++i) {
            // Lightweight path: assumes updateConstraintState already computed effectivePenetration this frame.
            // We intentionally avoid any temporal state mutation here so repeated iterations do not distort contact state.
            // If updateConstraintState was not called (edge case), fall back to raw penetration.
        
            // (World-space points retained for potential future true fresh penetration recompute; currently penetration cached.)
            vec3 world_rA = rotate(bodyA->orientation, contacts[i].rA);
            vec3 world_rB = rotate(bodyB->orientation, contacts[i].rB);
            (void)world_rA; (void)world_rB;
        
            float effectivePen = contacts[i].effectivePenetration;
            if (effectivePen <= 0.0f) {
                // Not yet initialized: derive a capped effective penetration directly.
                float penRaw = contacts[i].penetration;
                if (penRaw > 0.0f) {
                    const float INIT_CAP = 0.008f; // mirror steady-state cap
                    effectivePen = (penRaw > INIT_CAP)? INIT_CAP : penRaw;
                }
                contacts[i].effectivePenetration = effectivePen;
            }
            // Constraint function
            C[i*3 + 0] = max(0.0f, effectivePen - PENETRATION_SLOP);
            C[i*3 + 1] = 0.0f; // positional friction rows have zero target position error; we only bound impulses
            C[i*3 + 2] = 0.0f;
            fmin[i*3 + 0] = 0.0f;
            fmax[i*3 + 0] = FLT_MAX;
            bool frictionEnabled = (stiffness[i*3 + 1] > 0.0f);
            if (frictionEnabled) {
                float friction_limit = combinedFriction * fabsf(lambda[i*3 + 0]);
                fmin[i*3 + 1] = -friction_limit;
                fmax[i*3 + 1] =  friction_limit;
                fmin[i*3 + 2] = -friction_limit;
                fmax[i*3 + 2] =  friction_limit;
            } else {
                fmin[i*3 + 1] = fmax[i*3 + 1] = 0.0f;
                fmin[i*3 + 2] = fmax[i*3 + 2] = 0.0f;
            }

            // Sticking logic (uses cached tangential state)
            float friction_limit = combinedFriction * fabsf(lambda[i*3 + 0]);
            float tangent_lambda = sqrtf(lambda[i*3+1]*lambda[i*3+1] + lambda[i*3+2]*lambda[i*3+2]);
            contacts[i].stick = tangent_lambda < friction_limit && length(contacts[i].C0_t) < STICK_THRESH;

            if (bodyA && bodyA->solver && solver_should_trace(bodyA->solver)) {
                const Solver* sol = bodyA->solver;
                printf("TRACE_CONSTRAINT f=%d phase=%s body=%d m=%p ci=%d raw=%.6f eff0=%.6f eff=%.6f C=%.6f pen=%g lam=%g mem=%.6f memAge=%d spike=%d calls=%d\n",
                       sol->frameIndex,
                       sol->debugPhase?sol->debugPhase:"?",
                       sol->debugInvokingBodyId,
                       (void*)this,
                       i,
                       contacts[i].penetration,
                       contacts[i].effectivePenetration, // before spike unchanged in lightweight
                       contacts[i].effectivePenetration,
                       C[i*3+0],
                       penalty[i*3+0],
                       lambda[i*3+0],
                       contacts[i].depthMemory,
                       contacts[i].memoryAge,
                       contacts[i].spikeThisFrame?1:0,
                       computeCallsThisFrame);
            }
        }
    }

    // Heavy per-frame temporal state update: run exactly once before iterative solver passes
    void Manifold::updateConstraintState(float alpha) {
        computeCallsThisFrame++;
        if (bodyA && bodyA->solver) {
            bodyA->solver->diagConstraintComputeCalls++; // count also toward total for backward compatibility
            bodyA->solver->diagConstraintHeavyUpdateCalls++;
        }
        int frameIndex = bodyA->solver ? bodyA->solver->frameIndex : 0;
    for (int i = 0; i < numContacts; ++i) {
            float pen = contacts[i].penetration; // raw penetration

            // --- Adaptive penetration bias (decays over time) ---
            const float BASE_PEN_BIAS = 0.0025f; // initial target shallow interpenetration to expedite support formation
            const int BIAS_FADE_FRAMES = 480;    // fade to zero over 8 seconds @60Hz
            float biasScale = 0.0f;
            if (frameIndex < BIAS_FADE_FRAMES) biasScale = 1.0f - (float)frameIndex / (float)BIAS_FADE_FRAMES;
            float dynamicBias = BASE_PEN_BIAS * biasScale;
            // Height-based early amplification (only during first half of fade window)
            if (frameIndex < BIAS_FADE_FRAMES/2) {
                float h = fminf(bodyA->position.y, bodyB->position.y);
                if (h < 1.5f) {
                    float t = 1.0f - h/1.5f;
                    dynamicBias += 0.0010f * t * (1.0f - (float)frameIndex / (float)(BIAS_FADE_FRAMES/2));
                }
            }

            // Only enforce a floor while contact still stabilizing or early in simulation
            const int STABLE_DISABLE_BIAS_FRAMES = 90;
            if (pen > 0.0f && pen < dynamicBias && (contacts[i].stableFrames < STABLE_DISABLE_BIAS_FRAMES)) {
                pen = dynamicBias;
            }

            // Adaptive cap ramp
            float maxCapTarget = 0.008f;
            if (frameIndex < 180) {
                float ramp = (float)frameIndex / 180.0f;
                maxCapTarget = 0.003f + (0.008f - 0.003f) * ramp;
            }
            if (frameIndex > 300 && contacts[i].stableFrames > 120 && !contacts[i].penetrationClamped) {
                maxCapTarget = 0.0055f;
            }
            const float MAX_EFFECTIVE_PENETRATION = maxCapTarget;
            float effectivePen = (pen > MAX_EFFECTIVE_PENETRATION) ? MAX_EFFECTIVE_PENETRATION : pen;

            // Hysteresis memory clamp
            if (contacts[i].depthMemory > 0.0f) {
                float allowance = contacts[i].depthMemory + 0.0006f;
                if (effectivePen > allowance) {
                    float excess = effectivePen - allowance;
                    effectivePen = allowance + excess * 0.2f;
                }
            }

            // Spike cooldown decay
            if (contacts[i].spikeCooldown > 0) contacts[i].spikeCooldown--;

            // Growth damping & spike detection (effective)
            float prev = contacts[i].prevEffectivePen;
            float prevRaw = contacts[i].prevRawPen;
            if (effectivePen > 0.0f && prev > 0.0f) {
                float growth = (effectivePen - prev) / (prev + 1e-9f);
                if (growth > 0.5f) {
                    float factor = (growth > 1.5f) ? 0.25f : 0.5f;
                    effectivePen = prev + (effectivePen - prev) * factor;
                }
                if (growth > 2.0f) contacts[i].spikeThisFrame = true;
            }
            // Raw spike detection
            if (!contacts[i].spikeThisFrame && pen > 0.15f && prevRaw > 0.0f) {
                float rawGrowth = (pen - prevRaw) / (prevRaw + 1e-9f);
                if (rawGrowth > 0.5f || pen > 0.25f) contacts[i].spikeThisFrame = true;
            }
            // Early-frame pathological trim
            if (frameIndex < 120 && pen > MAX_EFFECTIVE_PENETRATION * 2.0f) {
                float excess = pen - MAX_EFFECTIVE_PENETRATION * 2.0f;
                pen = MAX_EFFECTIVE_PENETRATION * 2.0f + excess * 0.2f;
                effectivePen = (pen > MAX_EFFECTIVE_PENETRATION) ? MAX_EFFECTIVE_PENETRATION : pen;
            }
            // Temporal smoothing
            if (contacts[i].prevEffectivePen > 0.0f && effectivePen > contacts[i].prevEffectivePen) {
                const float MAX_DELTA = 0.0025f;
                float allowed = contacts[i].prevEffectivePen + MAX_DELTA;
                if (effectivePen > allowed) effectivePen = allowed;
            }
            // Settle snap around current (possibly fading) bias target
            if (dynamicBias > 0.0f) {
                if (effectivePen > dynamicBias && effectivePen < dynamicBias + 0.0025f) {
                    effectivePen = dynamicBias + (effectivePen - dynamicBias) * 0.3f;
                    if (effectivePen < dynamicBias + 0.0004f) effectivePen = dynamicBias + 0.0004f;
                }
            }

            contacts[i].effectivePenetration = effectivePen;
            contacts[i].penetrationClamped = (pen > MAX_EFFECTIVE_PENETRATION);
            contacts[i].prevEffectivePen = effectivePen;

            // Spike mitigation scaling + cooldown
            if (contacts[i].spikeThisFrame) {
                contacts[i].effectivePenetration *= 0.5f;
                contacts[i].spikeCooldown = 30;
            }
            if (contacts[i].spikeCooldown > 0) {
                float fade = (float)contacts[i].spikeCooldown / 30.0f;
                float scale = 0.7f + 0.3f * fade;
                contacts[i].effectivePenetration *= scale;
            }
            contacts[i].prevRawPen = pen; // record raw for next frame

            // Stability tracking for adaptive penalty relaxation
            const float STABLE_EFF_THRESH = fmaxf(dynamicBias, 0.00015f) + 0.0008f;
            if (effectivePen > 0.0f && effectivePen <= STABLE_EFF_THRESH) {
                contacts[i].stableFrames++;
                if (contacts[i].stableFrames == 60 || (contacts[i].depthMemory <= 0.0f && contacts[i].stableFrames >= 15)) {
                    contacts[i].depthMemory = effectivePen;
                    contacts[i].memoryAge = 0;
                } else if (contacts[i].depthMemory > 0.0f) {
                    // Freeze memory aging if bodies remain within a tiny vertical gap (quasi-contact)
                    bool freeze = false;
                    if (bodyA && bodyB) {
                        float gapY = fabsf(bodyB->position.y - bodyA->position.y) - (bodyA->size.y*0.5f + bodyB->size.y*0.5f);
                        if (gapY > 0 && gapY < COLLISION_MARGIN * 2.0f) freeze = true;
                    }
                    if (!freeze) contacts[i].memoryAge++;
                    if (effectivePen < contacts[i].depthMemory) {
                        contacts[i].depthMemory = contacts[i].depthMemory*0.9f + effectivePen*0.1f;
                    }
                }
            } else {
                contacts[i].stableFrames = 0;
                if (contacts[i].depthMemory > 0.0f) {
                    bool freeze = false;
                    if (bodyA && bodyB) {
                        float gapY = fabsf(bodyB->position.y - bodyA->position.y) - (bodyA->size.y*0.5f + bodyB->size.y*0.5f);
                        if (gapY > 0 && gapY < COLLISION_MARGIN * 2.0f) freeze = true;
                    }
                    if (!freeze) {
                        contacts[i].memoryAge++;
                        if (contacts[i].memoryAge > 240) { contacts[i].depthMemory = 0.0f; contacts[i].memoryAge = 0; }
                    }
                }
            }
            contacts[i].relaxPenalty = (contacts[i].stableFrames >= 120);

            // Persistence: if raw penetration vanished but we are still early or contact is shallow-stable, keep memory alive longer
            if (pen <= 0.0f && contacts[i].depthMemory > 0.0f) {
                // Do not immediately zero effectivePenetration; rely on memory for a grace window
                if (contacts[i].memoryAge < 60) {
                    contacts[i].effectivePenetration = fmaxf(contacts[i].effectivePenetration, contacts[i].depthMemory * 0.85f);
                }
            }

            // --- Shallow stable penalty ramp (increase stiffness instead of enforcing residual depth) ---
            if (contacts[i].stableFrames >= 45 && contacts[i].effectivePenetration > 0.0f && contacts[i].effectivePenetration < 0.0022f) {
                float& rowPenalty = penalty[i*3 + 0];
                if (rowPenalty < 120.0f) {
                    rowPenalty = fminf(rowPenalty * 1.10f + 1.0f, 120.0f);
                }
            }

            // Micro anti-creep: if extremely shallow and very stable, slightly bias effective penetration downward to counter slow sinking
            if (contacts[i].stableFrames > 180 && contacts[i].effectivePenetration < 0.0018f) {
                contacts[i].effectivePenetration *= 0.97f; // gentle decay toward zero
                if (contacts[i].effectivePenetration < 0.0005f) contacts[i].effectivePenetration = 0.0f;
            }

            // Grounded contact contribution: upward facing (normal.y > 0.6) & reasonably stable
            if (contacts[i].stableFrames > 20 && contacts[i].effectivePenetration < 0.004f) {
                if (contacts[i].normal.y > 0.6f) {
                    if (bodyA && bodyA->invMass > 0) bodyA->groundedContacts++;
                    if (bodyB && bodyB->invMass > 0) bodyB->groundedContacts++;
                    // Approximate support load contribution using penalty * effectivePen (proxy for normal reaction magnitude)
                    float load = penalty[i*3 + 0] * contacts[i].effectivePenetration;
                    if (bodyA && bodyA->invMass > 0) bodyA->supportLoad += load * 0.5f;
                    if (bodyB && bodyB->invMass > 0) bodyB->supportLoad += load * 0.5f;
                }
            }

            if (bodyA && bodyA->solver && solver_should_trace(bodyA->solver)) {
                const Solver* sol = bodyA->solver;
                printf("TRACE_CONSTRAINT f=%d phase=update body=%d m=%p ci=%d raw=%.6f eff=%.6f C=%.6f pen=%g lam=%g mem=%.6f memAge=%d spike=%d calls=%d\n",
                       sol->frameIndex,
                       sol->debugInvokingBodyId,
                       (void*)this,
                       i,
                       pen,
                       contacts[i].effectivePenetration,
                       max(0.0f, contacts[i].effectivePenetration - PENETRATION_SLOP),
                       penalty[i*3+0],
                       lambda[i*3+0],
                       contacts[i].depthMemory,
                       contacts[i].memoryAge,
                       contacts[i].spikeThisFrame?1:0,
                       computeCallsThisFrame);
            }
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
    
    float sign = (body == bodyA) ? 1.0f : -1.0f; // Follow 2D reference: +normal for A, -normal for B
    
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