/*
 * manifold.cpp - 3D AVBD Physics Engine
 *
 * Hybrid manifold constraints:
 * - Keeps the current row-based solver interface.
 * - Improves contact persistence (lambda + penalty carryover).
 * - Uses stable tangent basis generation and friction cone projection.
 */

#include "solver.h"

#include <cfloat>
#include <cmath>

namespace {

constexpr float NORMAL_CONTACT_MARGIN = 0.01f;
constexpr float STICK_ANCHOR_MAX_DRIFT = 0.015f;
constexpr float STICK_NORMAL_MIN_DOT = 0.995f;

static vec3 worldContactPoint(const Rigid* body, const vec3& localPoint)
{
    return body->position + rotate(body->orientation, localPoint);
}

static vec3 normalizeSafe(const vec3& v, const vec3& fallback)
{
    float lenSq = lengthSq(v);
    if (lenSq < VEC_EPSILON) {
        return fallback;
    }
    return v / sqrtf(lenSq);
}

static void buildContactBasis(const vec3& normalIn, vec3& normal, vec3& tangent1, vec3& tangent2)
{
    normal = normalizeSafe(normalIn, vec3(0.0f, 1.0f, 0.0f));

    if (fabsf(normal.x) >= fabsf(normal.z)) {
        tangent1 = vec3(-normal.y, normal.x, 0.0f);
    } else {
        tangent1 = vec3(0.0f, -normal.z, normal.y);
    }
    tangent1 = normalizeSafe(tangent1, vec3(1.0f, 0.0f, 0.0f));
    tangent2 = normalizeSafe(cross(normal, tangent1), vec3(0.0f, 0.0f, 1.0f));
}

} // namespace

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB)
    : Force(solver, bodyA, bodyB)
    , numContacts(0)
    , combinedFriction(0.0f)
{
    for (int i = 0; i < 4; ++i) {
        contacts[i].C0_n = 0.0f;
        contacts[i].C0_t = vec3();
        contacts[i].stick = false;
    }
}

int Manifold::getRowCount() const
{
    return numContacts * 3; // 1 normal + 2 tangents per contact
}

bool Manifold::initialize()
{
    combinedFriction = sqrtf(bodyA->friction * bodyB->friction);

    // Cache previous manifold rows for warmstart carryover.
    Contact oldContacts[4];
    float oldLambda[12];
    float oldPenalty[12];
    bool oldUsed[4] = {false, false, false, false};
    int oldNumContacts = numContacts;

    for (int i = 0; i < oldNumContacts; ++i) {
        oldContacts[i] = contacts[i];
        for (int k = 0; k < 3; ++k) {
            int row = i * 3 + k;
            oldLambda[row] = lambda[row];
            oldPenalty[row] = penalty[row];
        }
    }

    // Build new contact manifold.
    numContacts = Manifold::collide(bodyA, bodyB, contacts, false);
    if (numContacts == 0) {
        return false;
    }

    // Initialize all rows and transfer warmstart data by exact feature id.
    for (int i = 0; i < numContacts; ++i) {
        int base = i * 3;
        for (int k = 0; k < 3; ++k) {
            int row = base + k;
            stiffness[row] = FLT_MAX;
            lambda[row] = 0.0f;
            penalty[row] = PENALTY_MIN;
            motor[row] = 0.0f;
        }
        contacts[i].stick = false;

        int best = -1;

        for (int j = 0; j < oldNumContacts; ++j) {
            if (oldUsed[j]) {
                continue;
            }
            if (contacts[i].feature.value == oldContacts[j].feature.value) {
                best = j;
                break;
            }
        }

        if (best >= 0) {
            oldUsed[best] = true;

            for (int k = 0; k < 3; ++k) {
                int row = base + k;
                int oldRow = best * 3 + k;
                lambda[row] = oldLambda[oldRow];
                penalty[row] = clamp(oldPenalty[oldRow], PENALTY_MIN, PENALTY_MAX);
            }

            bool reuseStickAnchors = false;
            if (oldContacts[best].stick) {
                vec3 newNormal = normalizeSafe(contacts[i].normal, vec3(0.0f, 1.0f, 0.0f));
                vec3 oldNormal = normalizeSafe(oldContacts[best].normal, newNormal);
                float normalDot = dot(newNormal, oldNormal);

                vec3 oldMid = (worldContactPoint(bodyA, oldContacts[best].rA) + worldContactPoint(bodyB, oldContacts[best].rB)) * 0.5f;
                vec3 newMid = (worldContactPoint(bodyA, contacts[i].rA) + worldContactPoint(bodyB, contacts[i].rB)) * 0.5f;
                float driftSq = lengthSq(newMid - oldMid);

                reuseStickAnchors = (normalDot >= STICK_NORMAL_MIN_DOT)
                    && (driftSq <= STICK_ANCHOR_MAX_DRIFT * STICK_ANCHOR_MAX_DRIFT);
            }

            contacts[i].stick = oldContacts[best].stick && reuseStickAnchors;

            if (reuseStickAnchors) {
                contacts[i].rA = oldContacts[best].rA;
                contacts[i].rB = oldContacts[best].rB;
            }
        }

        // Cache q- violation for alpha stabilization during this step.
        vec3 normal, tangent1, tangent2;
        buildContactBasis(contacts[i].normal, normal, tangent1, tangent2);
        contacts[i].normal = normal;

        vec3 pA = worldContactPoint(bodyA, contacts[i].rA);
        vec3 pB = worldContactPoint(bodyB, contacts[i].rB);
        vec3 delta = pA - pB;
        // Use a small speculative margin to keep resting contacts active.
        contacts[i].C0_n = dot(delta, normal) - NORMAL_CONTACT_MARGIN;
        contacts[i].C0_t.x = dot(delta, tangent1);
        contacts[i].C0_t.y = dot(delta, tangent2);
        contacts[i].C0_t.z = 0.0f;
        contacts[i].penetration = max(0.0f, -dot(delta, normal));
    }

    return true;
}

void Manifold::computeConstraint(float alpha)
{
    float biasScale = clamp(1.0f - alpha, 0.0f, 1.0f);

    for (int i = 0; i < numContacts; ++i) {
        int base = i * 3;

        vec3 normal, tangent1, tangent2;
        buildContactBasis(contacts[i].normal, normal, tangent1, tangent2);
        contacts[i].normal = normal;

        vec3 pA = worldContactPoint(bodyA, contacts[i].rA);
        vec3 pB = worldContactPoint(bodyB, contacts[i].rB);
        vec3 delta = pA - pB;

        // Preserve near-contact support to reduce stack/pyramid jitter.
        float separation = dot(delta, normal) - NORMAL_CONTACT_MARGIN;
        float slip1 = dot(delta, tangent1);
        float slip2 = dot(delta, tangent2);

        // Normal row (unilateral compression only).
        C[base + 0] = separation + biasScale * contacts[i].C0_n;
        fmin[base + 0] = -FLT_MAX;
        fmax[base + 0] = 0.0f;

        // Tangential rows (friction).
        C[base + 1] = slip1 + biasScale * contacts[i].C0_t.x;
        C[base + 2] = slip2 + biasScale * contacts[i].C0_t.y;

        // Use a trial normal force (same pass as savant's manifold update) to
        // set a dynamic friction cone bound.
        float warmNormalMagnitude = fabsf(min(lambda[base + 0], 0.0f));
        float trialNormal = penalty[base + 0] * C[base + 0] + lambda[base + 0];
        float trialNormalMagnitude = fabsf(min(trialNormal, 0.0f));
        float normalMagnitude = max(warmNormalMagnitude, trialNormalMagnitude);

        float mu = combinedFriction;
        if (!contacts[i].stick) {
            mu *= 0.9f; // Slight kinetic friction drop
        }
        float frictionLimit = mu * normalMagnitude;

        // Keep warmstarted tangential impulses inside the current cone.
        float lt1 = lambda[base + 1];
        float lt2 = lambda[base + 2];
        float tanMag = sqrtf(lt1 * lt1 + lt2 * lt2);
        if (tanMag > frictionLimit && tanMag > 1.0e-8f) {
            float s = frictionLimit / tanMag;
            lambda[base + 1] *= s;
            lambda[base + 2] *= s;
        }

        fmin[base + 1] = -frictionLimit;
        fmax[base + 1] = frictionLimit;
        fmin[base + 2] = -frictionLimit;
        fmax[base + 2] = frictionLimit;

        float slipMagSq = C[base + 1] * C[base + 1] + C[base + 2] * C[base + 2];
        float tanLambdaSq = lambda[base + 1] * lambda[base + 1] + lambda[base + 2] * lambda[base + 2];
        float stickThreshSq = STICK_THRESH * STICK_THRESH;
        contacts[i].stick = (slipMagSq <= stickThreshSq) && (tanLambdaSq <= frictionLimit * frictionLimit + 1.0e-8f);

        contacts[i].penetration = max(0.0f, -dot(delta, normal));
    }
}

void Manifold::computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const
{
    int contactIdx = row / 3;
    int type = row % 3;
    const Contact& contact = contacts[contactIdx];

    vec3 normal, tangent1, tangent2;
    buildContactBasis(contact.normal, normal, tangent1, tangent2);

    vec3 basis;
    if (type == 0) {
        basis = normal;
    } else if (type == 1) {
        basis = tangent1;
    } else {
        basis = tangent2;
    }

    float sign = (body == bodyA) ? 1.0f : -1.0f;
    vec3 worldR = (body == bodyA) ? rotate(bodyA->orientation, contact.rA)
                                  : rotate(bodyB->orientation, contact.rB);

    J_linear = basis * sign;
    J_angular = cross(worldR, basis) * sign;
}

void Manifold::draw() const
{
    if (!SHOW_CONTACTS) {
        return;
    }

    glDisable(GL_LIGHTING);
    glPointSize(6.0f);
    glLineWidth(2.0f);

    for (int i = 0; i < numContacts; ++i) {
        vec3 pA = worldContactPoint(bodyA, contacts[i].rA);
        vec3 pB = worldContactPoint(bodyB, contacts[i].rB);
        vec3 pMid = (pA + pB) * 0.5f;

        if (contacts[i].stick) {
            glColor3f(1.0f, 1.0f, 0.0f); // sticking: yellow
        } else {
            glColor3f(0.8f, 0.2f, 0.8f); // sliding: purple
        }

        glBegin(GL_POINTS);
        glVertex3fv(&pMid.x);
        glEnd();

        glColor3f(1.0f, 0.2f, 0.2f);
        glBegin(GL_LINES);
        glVertex3fv(&pMid.x);
        vec3 end = pMid + contacts[i].normal * 0.5f;
        glVertex3fv(&end.x);
        glEnd();
    }

    glEnable(GL_LIGHTING);
}
