/*
* joint.cpp - 3D AVBD Physics Engine
*
* Implements the 3D "weld" joint logic.
*/

#include "joint.h" // Even if this file will be part of a larger cpp, it's good practice
#include <float.h>

// Constructor for connecting two bodies
Joint::Joint(Solver* solver, Rigid* bodyA, Rigid* bodyB,
             const vec3& localAnchorA, const vec3& localAnchorB,
             float linearStiffness, float angularStiffness, float motor, float fracture)
    : Force(solver, bodyA, bodyB), rA(localAnchorA), rB(localAnchorB)
{
    // The initial relative orientation is captured when the joint is created.
    // q_rel = conjugate(q_A) * q_B
    quat orientationA = bodyA ? bodyA->orientation : quat();
    initialRelativeOrientation = conjugate(orientationA) * bodyB->orientation;
    
    // Assign stiffness to all 6 rows
    for (int i = 0; i < 3; ++i) { // Linear rows
        stiffness[i] = linearStiffness;
        lambda[i] = 0;
        penalty[i] = PENALTY_MIN;
    }
    for (int i = 3; i < 6; ++i) { // Angular rows
        stiffness[i] = angularStiffness;
        lambda[i] = 0;
        penalty[i] = PENALTY_MIN;
    }
    
    // TODO: Motors and fracture would apply to specific angular axes.
    // For a simple weld joint, these are less common.
    this->angularStiffness = angularStiffness;
    this->angularMotor = motor;
    this->angularFracture = fracture;
}

// Constructor for connecting a body to a fixed point in the world
Joint::Joint(Solver* solver, Rigid* bodyB,
             const vec3& worldAnchor,
             float linearStiffness, float angularStiffness, float motor, float fracture)
    : Force(solver, nullptr, bodyB) // bodyA is null for world constraints
{
    // The local anchor on the world is just the world anchor itself.
    this->rA = worldAnchor;
    // The local anchor on the body is the world anchor transformed into its local space.
    this->rB = transpose(mat3_from_quat(bodyB->orientation)) * (worldAnchor - bodyB->position);
    
    // The initial relative orientation is just the body's world orientation.
    this->initialRelativeOrientation = bodyB->orientation;

    // Assign stiffness (same as other constructor)
    for (int i = 0; i < 3; ++i) { stiffness[i] = linearStiffness; lambda[i] = 0; penalty[i] = PENALTY_MIN; }
    for (int i = 3; i < 6; ++i) { stiffness[i] = angularStiffness; lambda[i] = 0; penalty[i] = PENALTY_MIN; }
    this->angularStiffness = angularStiffness;
    this->angularMotor = motor;
    this->angularFracture = fracture;
}


bool Joint::initialize() {
    // Joints are always active once created, so we just return true.
    return true;
}

void Joint::computeConstraint(float dt) {
    // --- Linear Constraint Violation (Rows 0, 1, 2) ---
    // The violation is the current separation vector between the world-space anchor points.
    vec3 pA, pB;
    quat qA;

    if (bodyA) {
        qA = bodyA->orientation;
        pA = bodyA->position + rotate(qA, rA);
    } else {
        qA = quat(); // World has identity orientation
        pA = rA;     // For world joint, rA is the world anchor
    }
    pB = bodyB->position + rotate(bodyB->orientation, rB);

    vec3 linear_C = pA - pB;
    C[0] = linear_C.x;
    C[1] = linear_C.y;
    C[2] = linear_C.z;

    // --- Angular Constraint Violation (Rows 3, 4, 5) ---
    // The violation is the axis-angle vector representing the rotation
    // needed to get from the current relative orientation back to the initial one.
    quat currentRelativeOrientation = conjugate(qA) * bodyB->orientation;
    quat deltaOrientation = currentRelativeOrientation * conjugate(initialRelativeOrientation);
    
    // For a small rotation, the axis-angle vector is approx. 2 * a quaternion's vector part.
    // We want to drive this vector to zero.
    vec3 angular_C = vec3(deltaOrientation.x, deltaOrientation.y, deltaOrientation.z) * 2.0f;
    C[3] = angular_C.x;
    C[4] = angular_C.y;
    C[5] = angular_C.z;

    // All force limits are infinite for a hard weld joint, but could be set for springs.
    for (int i = 0; i < 6; ++i) {
        fmin[i] = -FLT_MAX;
        fmax[i] = FLT_MAX;
    }
}

void Joint::computeDerivatives(vec3& J_linear, vec3& J_angular, const Rigid* body, int row) const {
    J_linear = {0,0,0};
    J_angular = {0,0,0};

    float sign = (body == bodyA) ? 1.0f : -1.0f;
    // If bodyA is the world (nullptr), it can't move, so its Jacobian contribution is zero.
    if (body == bodyA && bodyA == nullptr) {
        return;
    }

    if (row < 3) { // --- Linear Jacobian ---
        // The linear part is a standard basis vector (e.g., {1,0,0} for the x-constraint).
        vec3 linear_axis = {0,0,0};
        linear_axis[row] = 1.0f;

        // The angular part of the jacobian is cross(r, linear_axis), where r is the world-space lever arm.
        vec3 r = (body == bodyA) ? rotate(body->orientation, rA) : rotate(body->orientation, rB);
        
        J_linear = linear_axis * sign;
        J_angular = cross(r, linear_axis) * sign;

    } else { // --- Angular Jacobian ---
        // For angular constraints, the linear part of the jacobian is zero.
        J_linear = {0,0,0};
        
        // The angular part is a standard basis vector.
        vec3 angular_axis = {0,0,0};
        angular_axis[row - 3] = 1.0f;
        
        J_angular = angular_axis * sign;
    }
}

void Joint::draw() const {
    // Draw a line between the two anchor points.
    vec3 pA, pB;
    if (bodyA) {
        pA = bodyA->position + rotate(bodyA->orientation, rA);
    } else {
        pA = rA;
    }
    pB = bodyB->position + rotate(bodyB->orientation, rB);

    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glColor3f(0.2f, 0.8f, 0.2f); // Green for joints

    glBegin(GL_LINES);
    glVertex3fv(&pA.x);
    glVertex3fv(&pB.x);
    glEnd();

    glEnable(GL_LIGHTING);
}