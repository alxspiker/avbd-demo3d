/*
* rigid.cpp - 3D AVBD Physics Engine
*
* CORRECTED: The constructor's signature now exactly matches the
* declaration in solver.h, respecting 'const' and references '&'.
*/

#include "solver.h"

// CORRECTED: Added 'const' and '&' to match the header declaration
Rigid::Rigid(Solver* solver, const vec3& size, float density, float friction, const vec3& pos, const quat& orient, const vec3& linVel, const vec3& angVel)
    : solver(solver),
      forces(0),
      next(0),
      position(pos),
      orientation(orient),
      linearVelocity(linVel),
      angularVelocity(angVel),
      prevLinearVelocity(linVel),
      prevAngularVelocity(angVel),
      size(size),
      friction(friction)
{
    // Add this body to the solver's linked list.
    next = solver->bodies;
    solver->bodies = this;

    // --- 3D Mass Property Calculation ---
    mass = size.x * size.y * size.z * density;
    invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    radius = length(size) * 0.5f;

    if (invMass > 0.0f) {
        float Ixx = (1.0f / 12.0f) * mass * (size.y * size.y + size.z * size.z);
        float Iyy = (1.0f / 12.0f) * mass * (size.x * size.x + size.z * size.z);
        float Izz = (1.0f / 12.0f) * mass * (size.x * size.x + size.y * size.y);
        invInertiaTensor = mat3(
            {1.0f / Ixx, 0, 0},
            {0, 1.0f / Iyy, 0},
            {0, 0, 1.0f / Izz}
        );
    } else {
        invInertiaTensor = mat3({0,0,0}, {0,0,0}, {0,0,0});
    }
}

Rigid::~Rigid()
{
    Rigid** p = &solver->bodies;
    while (*p != this) {
        p = &(*p)->next;
    }
    *p = next;
}

mat3 Rigid::getInvInertiaTensorWorld() const
{
    mat3 R = mat3_from_quat(orientation);
    return R * invInertiaTensor * transpose(R);
}

bool Rigid::isConstrainedTo(Rigid* other) const
{
    // This traversal logic correctly checks the dual-linked list
    for (Force* f = forces; f != 0; f = (f->bodyA == this) ? f->nextA : f->nextB) {
        if (f->bodyB == other || f->bodyA == other) {
            return true;
        }
    }
    return false;
}

void Rigid::draw() const
{
    static const GLfloat vertices[][3] = {
        {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5},
        {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5}
    };
    static const GLubyte indices[][4] = {
        {0, 1, 2, 3}, {1, 5, 6, 2}, {5, 4, 7, 6}, {4, 0, 3, 7}, {3, 2, 6, 7}, {4, 5, 1, 0}
    };
    static const GLfloat normals[][3] = {
        {0, 0, 1}, {1, 0, 0}, {0, 0, -1}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}
    };

    glPushMatrix();

    glTranslatef(position.x, position.y, position.z);
    
    mat3 R = mat3_from_quat(orientation);
    GLfloat matrix[16] = {
        R.cols[0].x, R.cols[0].y, R.cols[0].z, 0,
        R.cols[1].x, R.cols[1].y, R.cols[1].z, 0,
        R.cols[2].x, R.cols[2].y, R.cols[2].z, 0,
        0, 0, 0, 1
    };
    glMultMatrixf(matrix);
    
    glScalef(size.x, size.y, size.z);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glColor3f(0.8f, 0.7f, 0.6f);

    glBegin(GL_QUADS);
    for (int i = 0; i < 6; i++) {
        glNormal3fv(normals[i]);
        for (int j = 0; j < 4; j++) {
            glVertex3fv(vertices[indices[i][j]]);
        }
    }
    glEnd();
    
    glDisable(GL_LIGHTING);
    glColor3f(0.1f, 0.1f, 0.1f);
    glLineWidth(2.0f);
    for (int i = 0; i < 6; i++) {
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j < 4; j++) {
            glVertex3fv(vertices[indices[i][j]]);
        }
        glEnd();
    }
    glEnable(GL_LIGHTING);

    glPopMatrix();
}