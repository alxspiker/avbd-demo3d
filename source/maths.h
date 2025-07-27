/*
* maths.h - 3D Math Library for AVBD Engine
*
* CORRECTED: Added missing operator- for mat3.
* UPDATED: Added global operator* for float * vec3 to enable commutative scalar multiplication.
*/

#pragma once

#include <cmath>
#include <cfloat>

// Forward declarations
struct vec3;
struct quat;
struct mat3;

const float VEC_EPSILON = 1e-6f;

//------------------------------------------------------------------------------------------------
// vec3
//------------------------------------------------------------------------------------------------
struct vec3 {
    float x, y, z;
    vec3() : x(0), y(0), z(0) {}
    vec3(float scalar) : x(scalar), y(scalar), z(scalar) {}
    vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    float& operator[](int i) { return ((float*)this)[i]; }
    const float& operator[](int i) const { return ((float*)this)[i]; }
    vec3 operator-() const { return vec3(-x, -y, -z); }
    vec3 operator+(const vec3& rhs) const { return vec3(x + rhs.x, y + rhs.y, z + rhs.z); }
    vec3 operator-(const vec3& rhs) const { return vec3(x - rhs.x, y - rhs.y, z - rhs.z); }
    vec3 operator*(float s) const { return vec3(x * s, y * s, z * s); }
    vec3 operator/(float s) const { return vec3(x / s, y / s, z / s); }
    vec3& operator+=(const vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    vec3& operator-=(const vec3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    vec3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }
};

inline float dot(const vec3& a, const vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float lengthSq(const vec3& v) { return dot(v, v); }
inline float length(const vec3& v) { return sqrtf(lengthSq(v)); }
inline vec3 normalize(const vec3& v) { float len = length(v); if (len < VEC_EPSILON) return vec3(); return v / len; }
inline vec3 cross(const vec3& a, const vec3& b) { return vec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x ); }
inline vec3 abs(const vec3& v) { return vec3(fabsf(v.x), fabsf(v.y), fabsf(v.z)); }

// UPDATED: Added to allow float * vec3 (commutative with vec3 * float)
inline vec3 operator*(float s, const vec3& v) { return v * s; }


//------------------------------------------------------------------------------------------------
// quat
//------------------------------------------------------------------------------------------------
struct quat {
    float x, y, z, w;
    quat() : x(0), y(0), z(0), w(1) {}
    quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    quat(const vec3& axis, float angle) { float halfAngle = angle * 0.5f; float s = sinf(halfAngle); w = cosf(halfAngle); x = axis.x * s; y = axis.y * s; z = axis.z * s; }
    quat operator+(const quat& rhs) const { return quat(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w); }
    quat operator-(const quat& rhs) const { return quat(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w); }
};

inline quat operator*(const quat& q, float s) { return quat(q.x * s, q.y * s, q.z * s, q.w * s); }
inline quat normalize(const quat& q) { float magSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w; if (magSq < VEC_EPSILON) return quat(); return q * (1.0f / sqrtf(magSq)); }
inline quat conjugate(const quat& q) { return quat(-q.x, -q.y, -q.z, q.w); }
inline quat operator*(const quat& q1, const quat& q2) { return quat( q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y, q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x, q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w, q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z ); }
inline vec3 rotate(const quat& q, const vec3& v) { vec3 q_vec(q.x, q.y, q.z); vec3 t = cross(q_vec, v) * 2.0f; return v + t * q.w + cross(q_vec, t); }


//------------------------------------------------------------------------------------------------
// mat3
//------------------------------------------------------------------------------------------------
struct mat3 {
    vec3 cols[3];
    mat3() { cols[0] = {1,0,0}; cols[1] = {0,1,0}; cols[2] = {0,0,1}; }
    mat3(const vec3& c0, const vec3& c1, const vec3& c2) { cols[0] = c0; cols[1] = c1; cols[2] = c2; }
    static mat3 diagonal(float d) { return mat3({d,0,0}, {0,d,0}, {0,0,d}); }
    static mat3 diagonal(const vec3& v) { return mat3({v.x,0,0}, {0,v.y,0}, {0,0,v.z}); }
};

inline mat3 transpose(const mat3& m) { return mat3( vec3(m.cols[0].x, m.cols[1].x, m.cols[2].x), vec3(m.cols[0].y, m.cols[1].y, m.cols[2].y), vec3(m.cols[0].z, m.cols[1].z, m.cols[2].z) ); }
inline vec3 operator*(const mat3& m, const vec3& v) { return m.cols[0] * v.x + m.cols[1] * v.y + m.cols[2] * v.z; }
inline mat3 operator*(const mat3& a, const mat3& b) { return mat3(a * b.cols[0], a * b.cols[1], a * b.cols[2]); }
inline mat3 operator+(const mat3& a, const mat3& b) { return mat3(a.cols[0] + b.cols[0], a.cols[1] + b.cols[1], a.cols[2] + b.cols[2]); }
inline mat3 operator*(const mat3& m, float s) { return mat3(m.cols[0] * s, m.cols[1] * s, m.cols[2] * s); }
inline mat3 outer_product(const vec3& a, const vec3& b) { return mat3(b * a.x, b * a.y, b * a.z); }
inline mat3 mat3_from_quat(const quat& q) { float xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z; float xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z; float wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z; return mat3( vec3(1-2*(yy+zz),   2*(xy+wz),     2*(xz-wy)), vec3(2*(xy-wz),     1-2*(xx+zz),   2*(yz+wx)), vec3(2*(xz+wy),     2*(yz-wx),     1-2*(xx+yy)) ); }
inline mat3& operator+=(mat3& a, const mat3& b) { a.cols[0] += b.cols[0]; a.cols[1] += b.cols[1]; a.cols[2] += b.cols[2]; return a; }

// CORRECTED: Added the missing subtraction operator for mat3
inline mat3 operator-(const mat3& a, const mat3& b) {
    return mat3(a.cols[0] - b.cols[0], a.cols[1] - b.cols[1], a.cols[2] - b.cols[2]);
}
// CORRECTED: Added division by scalar for use in spring.cpp
inline mat3 operator/(const mat3& m, float s) {
    return mat3(m.cols[0] / s, m.cols[1] / s, m.cols[2] / s);
}

// Generic Math & Solver
inline float min(float a, float b) { return a < b ? a : b; }
inline float max(float a, float b) { return a > b ? a : b; }
inline float clamp(float x, float a, float b) { return max(a, min(b, x)); }
inline vec3 solve(const mat3& A, const vec3& b) { vec3 L0=A.cols[0]; if(fabsf(L0.x)<FLT_EPSILON)return vec3(); float d0=L0.x; float L10=L0.y/d0; float L20=L0.z/d0; vec3 L1=A.cols[1]-L0*L10; if(fabsf(L1.y)<FLT_EPSILON)return vec3(); float d1=L1.y; float L21=L1.z/d1; vec3 L2=A.cols[2]-L0*L20-L1*L21; if(fabsf(L2.z)<FLT_EPSILON)return vec3(); float d2=L2.z; vec3 y; y.x=b.x; y.y=b.y-L10*y.x; y.z=b.z-L20*y.x-L21*y.y; vec3 z; z.x=y.x/d0; z.y=y.y/d1; z.z=y.z/d2; vec3 x; x.z=z.z; x.y=z.y-L21*x.z; x.x=z.x-L10*x.y-L20*x.z; return x; }