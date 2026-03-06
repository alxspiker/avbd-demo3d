/*
 * collision.cpp - 3D AVBD Physics Engine
 *
 * Hybrid collision pipeline:
 * - Keeps the current solver/manifold interface.
 * - Uses a robust SAT axis selection with face/edge contact generation.
 * - Produces stable, de-duplicated contact points and feature ids for
 *   warm-starting.
 */

#include "solver.h"

#include <cfloat>
#include <cmath>

namespace {

constexpr int MAX_CONTACTS = 4;
constexpr int MAX_POLY_VERTS = 16;
constexpr float SAT_AXIS_EPSILON = 1.0e-6f;
constexpr float PLANE_EPSILON = 1.0e-5f;
constexpr float CONTACT_MERGE_DIST_SQ = 1.0e-6f;
constexpr float CONTACT_PERSISTENCE_MARGIN = COLLISION_MARGIN;

enum AxisType {
    AXIS_FACE_A = 0,
    AXIS_FACE_B = 1,
    AXIS_EDGE = 2
};

struct OBB {
    vec3 center;
    vec3 half;
    vec3 axis[3];
};

struct SatAxis {
    AxisType type;
    int indexA;
    int indexB;
    float separation; // > 0 means separated
    vec3 normalAB;    // Unit normal pointing from A toward B
    bool valid;
};

struct FaceFrame {
    int axisIndex;
    vec3 normal; // Outward normal from reference face
    vec3 center;
    vec3 u;
    vec3 v;
    float extentU;
    float extentV;
};

static OBB makeOBB(const Rigid* body)
{
    OBB box{};
    box.center = body->position;
    box.half = body->size * 0.5f;
    mat3 R = mat3_from_quat(body->orientation);
    box.axis[0] = R.cols[0];
    box.axis[1] = R.cols[1];
    box.axis[2] = R.cols[2];
    return box;
}

static float absDot(const vec3& a, const vec3& b)
{
    return fabsf(dot(a, b));
}

static void getFaceAxes(const OBB& box, int axisIndex, vec3& u, vec3& v, float& extentU, float& extentV)
{
    if (axisIndex == 0) {
        u = box.axis[1];
        v = box.axis[2];
        extentU = box.half.y;
        extentV = box.half.z;
    } else if (axisIndex == 1) {
        u = box.axis[0];
        v = box.axis[2];
        extentU = box.half.x;
        extentV = box.half.z;
    } else {
        u = box.axis[0];
        v = box.axis[1];
        extentU = box.half.x;
        extentV = box.half.y;
    }
}

static void buildFaceFrame(const OBB& box, int axisIndex, const vec3& outwardNormal, FaceFrame& frame)
{
    float sign = dot(outwardNormal, box.axis[axisIndex]) >= 0.0f ? 1.0f : -1.0f;
    frame.axisIndex = axisIndex;
    frame.normal = box.axis[axisIndex] * sign;
    frame.center = box.center + frame.normal * box.half[axisIndex];
    getFaceAxes(box, axisIndex, frame.u, frame.v, frame.extentU, frame.extentV);
}

static int chooseIncidentFaceAxis(const OBB& box, const vec3& referenceNormal)
{
    int axis = 0;
    float best = -FLT_MAX;

    for (int i = 0; i < 3; ++i) {
        float d = absDot(box.axis[i], referenceNormal);
        if (d > best) {
            best = d;
            axis = i;
        }
    }

    return axis;
}

static void buildIncidentFace(const OBB& box, int axisIndex, const vec3& referenceNormal, vec3 outVerts[4])
{
    float sign = dot(box.axis[axisIndex], referenceNormal) > 0.0f ? -1.0f : 1.0f;
    vec3 faceNormal = box.axis[axisIndex] * sign;
    vec3 faceCenter = box.center + faceNormal * box.half[axisIndex];

    vec3 u;
    vec3 v;
    float extentU;
    float extentV;
    getFaceAxes(box, axisIndex, u, v, extentU, extentV);

    outVerts[0] = faceCenter + u * extentU + v * extentV;
    outVerts[1] = faceCenter - u * extentU + v * extentV;
    outVerts[2] = faceCenter - u * extentU - v * extentV;
    outVerts[3] = faceCenter + u * extentU - v * extentV;
}

static int clipPolygonAgainstPlane(const vec3* inVerts, int inCount, const vec3& planeNormal, float planeOffset, vec3* outVerts)
{
    if (inCount <= 0) {
        return 0;
    }

    int outCount = 0;
    vec3 a = inVerts[inCount - 1];
    float da = dot(planeNormal, a) - planeOffset;

    for (int i = 0; i < inCount; ++i) {
        vec3 b = inVerts[i];
        float db = dot(planeNormal, b) - planeOffset;

        bool aInside = da <= PLANE_EPSILON;
        bool bInside = db <= PLANE_EPSILON;

        if (aInside != bInside) {
            float t = 0.0f;
            float denom = da - db;
            if (fabsf(denom) > SAT_AXIS_EPSILON) {
                t = clamp(da / denom, 0.0f, 1.0f);
            }

            if (outCount < MAX_POLY_VERTS) {
                outVerts[outCount++] = a + (b - a) * t;
            }
        }

        if (bInside && outCount < MAX_POLY_VERTS) {
            outVerts[outCount++] = b;
        }

        a = b;
        da = db;
    }

    return outCount;
}

static bool addContact(Rigid* bodyA, Rigid* bodyB, Manifold::Contact* contacts, int& contactCount,
                       vec3* contactMidpoints, const vec3& xA, const vec3& xB, int featureKey,
                       const vec3& normalBA)
{
    vec3 midpoint = (xA + xB) * 0.5f;

    for (int i = 0; i < contactCount; ++i) {
        vec3 d = midpoint - contactMidpoints[i];
        if (lengthSq(d) < CONTACT_MERGE_DIST_SQ) {
            return false;
        }
    }

    if (contactCount >= MAX_CONTACTS) {
        return false;
    }

    Manifold::Contact& c = contacts[contactCount];
    c.feature.value = featureKey;
    c.rA = rotate(conjugate(bodyA->orientation), xA - bodyA->position);
    c.rB = rotate(conjugate(bodyB->orientation), xB - bodyB->position);
    c.normal = normalBA;
    c.penetration = max(0.0f, -dot(xA - xB, normalBA));
    c.C0_n = 0.0f;
    c.C0_t = vec3();
    c.stick = false;

    contactMidpoints[contactCount] = midpoint;
    ++contactCount;
    return true;
}

static bool testAxis(const OBB& boxA, const OBB& boxB, const vec3& delta, const vec3& axis,
                     AxisType type, int indexA, int indexB, SatAxis& best)
{
    float lenSq = lengthSq(axis);
    if (lenSq < SAT_AXIS_EPSILON) {
        return true;
    }

    vec3 n = axis / sqrtf(lenSq);
    if (dot(n, delta) < 0.0f) {
        n = -n;
    }

    float distance = fabsf(dot(delta, n));

    float rA = boxA.half.x * absDot(n, boxA.axis[0])
        + boxA.half.y * absDot(n, boxA.axis[1])
        + boxA.half.z * absDot(n, boxA.axis[2]);

    float rB = boxB.half.x * absDot(n, boxB.axis[0])
        + boxB.half.y * absDot(n, boxB.axis[1])
        + boxB.half.z * absDot(n, boxB.axis[2]);

    float separation = distance - (rA + rB);
    // Keep near contacts alive to avoid manifold churn in resting stacks.
    if (separation > CONTACT_PERSISTENCE_MARGIN) {
        return false;
    }

    if (!best.valid || separation > best.separation) {
        best.valid = true;
        best.type = type;
        best.indexA = indexA;
        best.indexB = indexB;
        best.separation = separation;
        best.normalAB = n;
    }

    return true;
}

static void supportEdge(const OBB& box, int axisIndex, const vec3& dir, vec3& edgeA, vec3& edgeB)
{
    int axis1 = (axisIndex + 1) % 3;
    int axis2 = (axisIndex + 2) % 3;

    float sign1 = dot(dir, box.axis[axis1]) >= 0.0f ? 1.0f : -1.0f;
    float sign2 = dot(dir, box.axis[axis2]) >= 0.0f ? 1.0f : -1.0f;

    vec3 edgeCenter = box.center
        + box.axis[axis1] * (box.half[axis1] * sign1)
        + box.axis[axis2] * (box.half[axis2] * sign2);

    edgeA = edgeCenter - box.axis[axisIndex] * box.half[axisIndex];
    edgeB = edgeCenter + box.axis[axisIndex] * box.half[axisIndex];
}

static void closestPointsOnSegments(const vec3& p0, const vec3& p1, const vec3& q0, const vec3& q1,
                                    vec3& c0, vec3& c1)
{
    vec3 d1 = p1 - p0;
    vec3 d2 = q1 - q0;
    vec3 r = p0 - q0;
    float a = dot(d1, d1);
    float e = dot(d2, d2);
    float f = dot(d2, r);

    float s = 0.0f;
    float t = 0.0f;

    if (a <= SAT_AXIS_EPSILON && e <= SAT_AXIS_EPSILON) {
        c0 = p0;
        c1 = q0;
        return;
    }

    if (a <= SAT_AXIS_EPSILON) {
        t = clamp(f / e, 0.0f, 1.0f);
    } else {
        float c = dot(d1, r);
        if (e <= SAT_AXIS_EPSILON) {
            s = clamp(-c / a, 0.0f, 1.0f);
        } else {
            float b = dot(d1, d2);
            float denom = a * e - b * b;

            if (fabsf(denom) > SAT_AXIS_EPSILON) {
                s = clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }

            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c0 = p0 + d1 * s;
    c1 = q0 + d2 * t;
}

static int buildFaceManifold(Rigid* bodyA, Rigid* bodyB, const OBB& boxA, const OBB& boxB,
                             bool referenceIsA, int referenceAxis, const vec3& normalAB,
                             Manifold::Contact* contacts)
{
    const OBB& referenceBox = referenceIsA ? boxA : boxB;
    const OBB& incidentBox = referenceIsA ? boxB : boxA;
    vec3 referenceOutward = referenceIsA ? normalAB : -normalAB;
    vec3 normalBA = -normalAB;

    FaceFrame referenceFace{};
    buildFaceFrame(referenceBox, referenceAxis, referenceOutward, referenceFace);

    int incidentAxis = chooseIncidentFaceAxis(incidentBox, referenceFace.normal);

    vec3 clip0[MAX_POLY_VERTS];
    vec3 clip1[MAX_POLY_VERTS];
    buildIncidentFace(incidentBox, incidentAxis, referenceFace.normal, clip0);
    int count = 4;

    vec3 n0 = referenceFace.u;
    float o0 = dot(n0, referenceFace.center) + referenceFace.extentU;
    count = clipPolygonAgainstPlane(clip0, count, n0, o0, clip1);
    if (!count) {
        return 0;
    }

    vec3 n1 = -referenceFace.u;
    float o1 = dot(n1, referenceFace.center) + referenceFace.extentU;
    count = clipPolygonAgainstPlane(clip1, count, n1, o1, clip0);
    if (!count) {
        return 0;
    }

    vec3 n2 = referenceFace.v;
    float o2 = dot(n2, referenceFace.center) + referenceFace.extentV;
    count = clipPolygonAgainstPlane(clip0, count, n2, o2, clip1);
    if (!count) {
        return 0;
    }

    vec3 n3 = -referenceFace.v;
    float o3 = dot(n3, referenceFace.center) + referenceFace.extentV;
    count = clipPolygonAgainstPlane(clip1, count, n3, o3, clip0);
    if (!count) {
        return 0;
    }

    int contactCount = 0;
    vec3 contactMidpoints[MAX_CONTACTS];
    int featurePrefix = (referenceIsA ? AXIS_FACE_A : AXIS_FACE_B) << 24;
    featurePrefix |= (referenceAxis & 0xFF) << 16;
    featurePrefix |= (incidentAxis & 0xFF) << 8;

    for (int i = 0; i < count && contactCount < MAX_CONTACTS; ++i) {
        vec3 pIncident = clip0[i];
        float distance = dot(pIncident - referenceFace.center, referenceFace.normal);
        // Accept a small gap so face contacts persist across tiny oscillations.
        if (distance > CONTACT_PERSISTENCE_MARGIN) {
            continue;
        }

        vec3 pReference = pIncident - referenceFace.normal * distance;
        vec3 xA = referenceIsA ? pReference : pIncident;
        vec3 xB = referenceIsA ? pIncident : pReference;

        // Deterministic feature key from local face coordinates (no ordering
        // dependence), so warmstart matching remains stable without fallback
        // proximity matching.
        vec3 rel = pReference - referenceFace.center;
        float uCoord = dot(rel, referenceFace.u);
        float vCoord = dot(rel, referenceFace.v);
        float uNorm = (referenceFace.extentU > SAT_AXIS_EPSILON) ? (uCoord / referenceFace.extentU) : 0.0f;
        float vNorm = (referenceFace.extentV > SAT_AXIS_EPSILON) ? (vCoord / referenceFace.extentV) : 0.0f;
        int qU = (int)floorf(clamp((uNorm + 1.0f) * 7.5f, 0.0f, 15.0f));
        int qV = (int)floorf(clamp((vNorm + 1.0f) * 7.5f, 0.0f, 15.0f));
        int featureKey = featurePrefix | ((qU & 0x0F) << 4) | (qV & 0x0F);

        addContact(bodyA, bodyB, contacts, contactCount, contactMidpoints, xA, xB, featureKey, normalBA);
    }

    return contactCount;
}

static int buildEdgeContact(Rigid* bodyA, Rigid* bodyB, const OBB& boxA, const OBB& boxB,
                            int axisA, int axisB, const vec3& normalAB, Manifold::Contact* contacts)
{
    vec3 a0;
    vec3 a1;
    vec3 b0;
    vec3 b1;
    supportEdge(boxA, axisA, normalAB, a0, a1);
    supportEdge(boxB, axisB, -normalAB, b0, b1);

    vec3 xA;
    vec3 xB;
    closestPointsOnSegments(a0, a1, b0, b1, xA, xB);

    int contactCount = 0;
    vec3 contactMidpoints[MAX_CONTACTS];
    int featureKey = (AXIS_EDGE << 24) | ((axisA & 0xFF) << 8) | (axisB & 0xFF);
    addContact(bodyA, bodyB, contacts, contactCount, contactMidpoints, xA, xB, featureKey, -normalAB);

    return contactCount;
}

} // namespace

int Manifold::collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts, bool flip)
{
    OBB boxA = makeOBB(bodyA);
    OBB boxB = makeOBB(bodyB);
    vec3 delta = boxB.center - boxA.center;

    SatAxis bestFace{};
    bestFace.separation = -FLT_MAX;
    bestFace.valid = false;

    SatAxis bestEdge{};
    bestEdge.separation = -FLT_MAX;
    bestEdge.valid = false;

    for (int i = 0; i < 3; ++i) {
        if (!testAxis(boxA, boxB, delta, boxA.axis[i], AXIS_FACE_A, i, -1, bestFace)) {
            return 0;
        }
    }

    for (int i = 0; i < 3; ++i) {
        if (!testAxis(boxA, boxB, delta, boxB.axis[i], AXIS_FACE_B, -1, i, bestFace)) {
            return 0;
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            vec3 axis = cross(boxA.axis[i], boxB.axis[j]);
            if (!testAxis(boxA, boxB, delta, axis, AXIS_EDGE, i, j, bestEdge)) {
                return 0;
            }
        }
    }

    if (!bestFace.valid) {
        return 0;
    }

    SatAxis best = bestFace;
    if (bestEdge.valid) {
        // Prefer edge-axis only when meaningfully better than face-axis to
        // reduce contact normal flicker.
        const float edgeRelTol = 0.95f;
        const float edgeAbsTol = 0.01f;
        if (edgeRelTol * bestEdge.separation > bestFace.separation + edgeAbsTol) {
            best = bestEdge;
        }
    }

    int count = 0;
    if (best.type == AXIS_EDGE) {
        count = buildEdgeContact(bodyA, bodyB, boxA, boxB, best.indexA, best.indexB, best.normalAB, contacts);
    } else if (best.type == AXIS_FACE_A) {
        count = buildFaceManifold(bodyA, bodyB, boxA, boxB, true, best.indexA, best.normalAB, contacts);
    } else {
        count = buildFaceManifold(bodyA, bodyB, boxA, boxB, false, best.indexB, best.normalAB, contacts);
    }

    if (flip) {
        for (int i = 0; i < count; ++i) {
            vec3 tmp = contacts[i].rA;
            contacts[i].rA = contacts[i].rB;
            contacts[i].rB = tmp;
            contacts[i].normal = -contacts[i].normal;
        }
    }

    return count;
}
