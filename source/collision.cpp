/*
* collision.cpp - 3D AVBD Physics Engine
*
* CORRECTED: Re-established working clipping logic and ensured contact
* normals are consistently generated to point from body B to body A.
*/

#include "solver.h"
#include <float.h>
#include <vector>
#include <algorithm>
#include <cmath> // For fabsf
#include <cstdio> // For printf

// --- Helper Functions for 3D Clipping ---

static void findFaceVertices(std::vector<vec3>& faceVertices, const Rigid* box, const vec3& worldNormal, bool incident) {
    mat3 R = mat3_from_quat(box->orientation);
    mat3 Rt = transpose(R);
    vec3 localNormal = Rt * (incident ? -worldNormal : worldNormal);

    vec3 absNormal = { fabsf(localNormal.x), fabsf(localNormal.y), fabsf(localNormal.z) };
    int primaryAxis = 0;
    float maxVal = absNormal.x;
    if (absNormal.y > maxVal) { maxVal = absNormal.y; primaryAxis = 1; }
    if (absNormal.z > maxVal) { primaryAxis = 2; }
    
    vec3 halfSize = box->size * 0.5f;
    float sign = (localNormal[primaryAxis] > 0) ? 1.0f : -1.0f;
    
    vec3 v[4];
    if (primaryAxis == 0) {
        v[0] = { sign * halfSize.x,  halfSize.y,  halfSize.z }; v[1] = { sign * halfSize.x, -halfSize.y,  halfSize.z };
        v[2] = { sign * halfSize.x, -halfSize.y, -halfSize.z }; v[3] = { sign * halfSize.x,  halfSize.y, -halfSize.z };
    } else if (primaryAxis == 1) {
        v[0] = {  halfSize.x, sign * halfSize.y,  halfSize.z }; v[1] = {  halfSize.x, sign * halfSize.y, -halfSize.z };
        v[2] = { -halfSize.x, sign * halfSize.y, -halfSize.z }; v[3] = { -halfSize.x, sign * halfSize.y,  halfSize.z };
    } else {
        v[0] = {  halfSize.x,  halfSize.y, sign * halfSize.z }; v[1] = { -halfSize.x,  halfSize.y, sign * halfSize.z };
        v[2] = { -halfSize.x, -halfSize.y, sign * halfSize.z }; v[3] = {  halfSize.x, -halfSize.y, sign * halfSize.z };
    }

    faceVertices.resize(4);
    for (int i = 0; i < 4; ++i) {
        faceVertices[i] = box->position + R * v[i];
    }
}

int clipPolygon(const std::vector<vec3>& inVertices, std::vector<vec3>& outVertices, const vec3& planeNormal, float planeOffset) {
    outVertices.clear();
    if (inVertices.empty()) return 0;
    
    vec3 s = inVertices.back();
    float s_dist = dot(planeNormal, s) - planeOffset;

    for (const auto& e : inVertices) {
        float e_dist = dot(planeNormal, e) - planeOffset;
        if (s_dist <= 0) {
            outVertices.push_back(s);
        }
        if ((s_dist * e_dist) < 0) {
            float t = s_dist / (s_dist - e_dist);
            outVertices.push_back(s + (e - s) * t);
        }
        s = e;
        s_dist = e_dist;
    }
    return outVertices.size();
}

int Manifold::collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts, bool flip) {
    // Adjusted: moderate allowance plus curved speculative penetration for early, stable support
    const float RESTING_ALLOWANCE = 0.05f; // 5cm window to begin seeding support
    const float SPEC_MIN = 0.0015f;        // base synthetic depth component (scaled by t)
    const float SPEC_DEPTH = 0.0050f;      // additional depth component (scaled by t^2)
    // Velocity reference used to attenuate speculative depth for fast approaches
    const float SPEC_VREF = 6.0f;          // m/s; above this, speculative depth scales down
    // Access frame index via solver pointer (bodies share solver)
    int frameIndex = bodyA->solver ? bodyA->solver->frameIndex : 0;
    mat3 R_A = mat3_from_quat(bodyA->orientation);
    mat3 R_B = mat3_from_quat(bodyB->orientation);
    vec3 D = bodyB->position - bodyA->position;

    float minPenetration = FLT_MAX;
    int bestAxis = -1;
    bool A_is_ref = true;

    // NOTE: We revert to immediate rejection per-axis (classic SAT) to avoid accumulating
    // large deep penetrations before first contact when a single axis has a sizable gap.
    // Test axes of A
    for (int i = 0; i < 3; ++i) {
        vec3 axis = R_A.cols[i];
        float ra = bodyA->size[i] * 0.5f;
        float rb = fabsf(dot(axis, R_B.cols[0])) * bodyB->size.x * 0.5f + 
                   fabsf(dot(axis, R_B.cols[1])) * bodyB->size.y * 0.5f + 
                   fabsf(dot(axis, R_B.cols[2])) * bodyB->size.z * 0.5f;
        float penetration = ra + rb - fabsf(dot(axis, D));
        if (penetration < -RESTING_ALLOWANCE) return 0; // definitive separation
        if (penetration < 0) {
            float gap = -penetration;
            float t = 1.0f - gap / RESTING_ALLOWANCE; // t in [0,1]
            // Curved profile: stronger near contact (t^2) + mild linear component, goes to 0 at allowance edge
            float speculative = (SPEC_MIN * t) + (SPEC_DEPTH * t * t);
            // Velocity attenuation: project relative velocity onto axis to scale speculative depth
            vec3 relV = bodyB->linearVelocity - bodyA->linearVelocity;
            float vAlong = fabsf(dot(relV, axis));
            float velScale = 1.0f - fminf(1.0f, vAlong / SPEC_VREF) * 0.85f; // up to 85% reduction at high speed
            speculative *= velScale;
            // Early-frame ramp: reduce speculative depth in the very first frames to prevent large seeded stacks
            if (frameIndex < 120) {
                float ramp = (float)frameIndex / 120.0f; // 0..1 over first 120 frames
                speculative *= (0.4f + 0.6f * ramp); // start at 40% strength
            }
            penetration = speculative;
        }
        if (penetration < minPenetration) { 
            minPenetration = penetration; 
            bestAxis = i; 
            A_is_ref = true;
        }
    }
    
    // Test axes of B
    for (int i = 0; i < 3; ++i) {
        vec3 axis = R_B.cols[i];
        float ra = fabsf(dot(axis, R_A.cols[0])) * bodyA->size.x * 0.5f + 
                   fabsf(dot(axis, R_A.cols[1])) * bodyA->size.y * 0.5f + 
                   fabsf(dot(axis, R_A.cols[2])) * bodyA->size.z * 0.5f;
        float rb = bodyB->size[i] * 0.5f;
        float penetration = ra + rb - fabsf(dot(axis, D));
        if (penetration < -RESTING_ALLOWANCE) return 0;
        if (penetration < 0) {
            float gap = -penetration;
            float t = 1.0f - gap / RESTING_ALLOWANCE;
            float speculative = (SPEC_MIN * t) + (SPEC_DEPTH * t * t);
            vec3 relV = bodyB->linearVelocity - bodyA->linearVelocity;
            float vAlong = fabsf(dot(relV, axis));
            float velScale = 1.0f - fminf(1.0f, vAlong / SPEC_VREF) * 0.85f;
            if (frameIndex < 120) {
                float ramp = (float)frameIndex / 120.0f;
                speculative *= (0.4f + 0.6f * ramp);
            }
            speculative *= velScale;
            penetration = speculative;
        }
        if (penetration < minPenetration) { 
            minPenetration = penetration; 
            bestAxis = i + 3; 
            A_is_ref = false; 
        }
    }
    
    // Test edge-edge cross products
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            vec3 axis = cross(R_A.cols[i], R_B.cols[j]);
            if (lengthSq(axis) < VEC_EPSILON) continue;
            axis = normalize(axis);
            float ra = fabsf(dot(axis, R_A.cols[0])) * bodyA->size.x * 0.5f + 
                       fabsf(dot(axis, R_A.cols[1])) * bodyA->size.y * 0.5f + 
                       fabsf(dot(axis, R_A.cols[2])) * bodyA->size.z * 0.5f;
            float rb = fabsf(dot(axis, R_B.cols[0])) * bodyB->size.x * 0.5f + 
                       fabsf(dot(axis, R_B.cols[1])) * bodyB->size.y * 0.5f + 
                       fabsf(dot(axis, R_B.cols[2])) * bodyB->size.z * 0.5f;
            float penetration = ra + rb - fabsf(dot(axis, D));
            if (penetration < -RESTING_ALLOWANCE) return 0;
            if (penetration < 0) {
                float gap = -penetration;
                float t = 1.0f - gap / RESTING_ALLOWANCE;
                float speculative = (SPEC_MIN * t) + (SPEC_DEPTH * t * t);
                vec3 relV = bodyB->linearVelocity - bodyA->linearVelocity;
                float vAlong = fabsf(dot(relV, axis));
                float velScale = 1.0f - fminf(1.0f, vAlong / SPEC_VREF) * 0.85f;
                if (frameIndex < 120) {
                    float ramp = (float)frameIndex / 120.0f;
                    speculative *= (0.4f + 0.6f * ramp);
                }
                speculative *= velScale;
                penetration = speculative;
            }
            if (penetration < minPenetration) { 
                minPenetration = penetration; 
                bestAxis = 6 + i * 3 + j; 
                A_is_ref = true; // Not strictly true, but convention
            }
        }
    }
    

    // For face-face contacts, prefer the larger/static object as reference
    // This ensures contact points are placed on the more stable surface
    if (bestAxis < 6) { // Face-face contact
        if (bodyB->invMass == 0.0f || (bodyA->invMass > 0.0f && length(bodyB->size) > length(bodyA->size))) {
            A_is_ref = false; // Use bodyB (ground) as reference
        }
    }
    
    Rigid *refBody = A_is_ref ? bodyA : bodyB;
    Rigid *incBody = A_is_ref ? bodyB : bodyA;

    vec3 refNormal;
    if (bestAxis < 3) { refNormal = R_A.cols[bestAxis]; }
    else if (bestAxis < 6) { refNormal = R_B.cols[bestAxis-3]; }
    else { int i = (bestAxis-6)/3; int j = (bestAxis-6)%3; refNormal = normalize(cross(R_A.cols[i], R_B.cols[j])); }

    // Ensure refNormal points from reference body towards incident body
    if (dot(refNormal, incBody->position - refBody->position) < 0) refNormal = -refNormal;

    std::vector<vec3> incidentFace;
    findFaceVertices(incidentFace, incBody, refNormal, true);

    std::vector<vec3> refFace;
    findFaceVertices(refFace, refBody, refNormal, false);
    
    // Early-phase clamp: limit extremely large penetration depth before translating face
    float depth = minPenetration;
    // Adaptive per-axis absolute penetration guard (prevents very deep raw overlaps entering solver)
    if (bodyA->solver) {
        // Ramp allowable depth early to avoid large initial overlaps forming tall stacks
        float axisLimit;
        if (bodyA->solver->frameIndex < 180) {
            float t = (float)bodyA->solver->frameIndex / 180.0f; // 0..1
            axisLimit = 0.06f + (0.09f - 0.06f) * t; // 6cm -> 9cm (tighter early window)
        } else {
            axisLimit = 0.09f; // steady state tightened from 12cm
        }
        if (depth > axisLimit) {
            depth = axisLimit;
            bodyA->solver->diagAxisClampCount++; // count how many manifolds required axis clamp
        }
        // Late-frame stricter clamp: after settling, keep new contacts from exceeding 3.5cm raw depth
        if (bodyA->solver->frameIndex > 300) {
            // Two-phase late tightening: 3.5cm after 300, 2.5cm after 360
            float lateLimit = (bodyA->solver->frameIndex > 360) ? 0.025f : 0.035f;
            if (depth > lateLimit) depth = lateLimit;
        }
    }
    if (bodyA->solver && bodyA->solver->frameIndex < 120) {
        const float EARLY_DEPTH_LIMIT = 0.18f; // meters
        if (depth > EARLY_DEPTH_LIMIT) depth = EARLY_DEPTH_LIMIT;
    }
    // Translate incident face to touching position with (possibly clamped) depth
    for (auto& v : incidentFace) {
        v += refNormal * depth;
    }
    
    // Compute clipping plane offset
    float planeOffset = dot(refFace[0], refNormal);

    // Sutherland-Hodgman Clipping
    std::vector<vec3> clipped = incidentFace;
    std::vector<vec3> temp;
    
    for (int i = 0; i < 4; ++i) {
        vec3 p1 = refFace[i];
        vec3 p2 = refFace[(i + 1) % 4];
        vec3 edge = p2 - p1;
        
        // --- FIX ---
        // The side normal for the clipping plane must point OUTWARDS from the reference face
        // because our clipper keeps points on the negative side. This logic achieves that.
        vec3 sideNormal = normalize(cross(edge, refNormal));
        vec3 toCenter = refFace[(i + 2) % 4] - p1;
        if (dot(sideNormal, toCenter) > 0) {
            sideNormal = -sideNormal;
        }

        float sideOffset = dot(p1, sideNormal);
        clipPolygon(clipped, temp, sideNormal, sideOffset);
        clipped = temp;
        temp.clear();
        if (clipped.empty()) break;
    }

    int numContacts = 0;
    for (const auto& v : clipped) {
    float separation = dot(v, refNormal) - planeOffset;
        if (separation <= COLLISION_MARGIN) {
            if (numContacts >= 8) break;
            
            // UPDATED: The normal must consistently point from Body B to Body A for the solver.
            contacts[numContacts].normal = (refBody == bodyA) ? -refNormal : refNormal;
            
            contacts[numContacts].penetration = depth;
            vec3 contactPoint = v - refNormal * separation;
            contacts[numContacts].rA = transpose(R_A) * (contactPoint - bodyA->position);
            contacts[numContacts].rB = transpose(R_B) * (contactPoint - bodyB->position);
            contacts[numContacts].feature.value = bestAxis;
            numContacts++;
        }
    }

    return numContacts;
}