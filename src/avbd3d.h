#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <limits>
#include <vector>

namespace avbd {
struct Vec3 {
    double x=0,y=0,z=0;
    Vec3()=default;
    Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){}
    Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
    Vec3 operator/(double s) const { return *this*(1/s); }
    Vec3& operator+=(Vec3 b) {x+=b.x;y+=b.y;z+=b.z;return *this;}
    Vec3& operator-=(Vec3 b) {return *this+=-b;}
    Vec3& operator*=(double s) {x*=s;y*=s;z*=s;return *this;}
    double operator[](int i) const {return i==0?x:(i==1?y:z);}
};
inline Vec3 operator*(double s,Vec3 v){return v*s;}
inline double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double length2(Vec3 v){return dot(v,v);}
double length(Vec3 v);
Vec3 normalized(Vec3 v);
struct Quat {
    double w=1,x=0,y=0,z=0;
    Quat()=default;
    Quat(double W,double X,double Y,double Z):w(W),x(X),y(Y),z(Z){}
    Quat operator*(Quat b) const;
    Quat conjugate() const {return {w,-x,-y,-z};}
    Quat unit() const;
    Vec3 rotate(Vec3 v) const;
    static Quat rotation(Vec3 axis,double radians);
    static Quat exp(Vec3 rotationVector);
    Vec3 log() const;
};

enum class Shape : uint8_t {Box,Sphere};
struct Body {
    Shape shape=Shape::Box;
    int id=0;
    Vec3 p, velocity, angularVelocity, half;
    Quat q;
    double mass=0, invMass=0, friction=0.6, restitution=0;
    Vec3 inertia,invInertia; // Principal moments of inertia in body space
    bool sleeping=false;
    double quietTime=0;
    bool dynamic() const {return invMass>0;}
};
struct Contact {
    Vec3 rA,rB; // Local-space witness points
    Vec3 n; // World normal B -> A, yielding C=(pA-pB) . n >= 0
    Vec3 initialDelta; // Relative anchor coordinates at beginning of step
    double lambdaN=0, lambdaT1=0,lambdaT2=0;
    double kN=1000,kT1=1000,kT2=1000;
    double impactSpeed=0; // Closing speed measured at the beginning of a new contact
    bool matched=false;
};
struct Manifold {int a=-1,b=-1; std::vector<Contact> contacts;};
struct DistanceJoint {
    int a=-1,b=-1;
    Vec3 anchorA,anchorB; // Body-local anchors
    double restLength=0;
    double stiffness=std::numeric_limits<double>::infinity();
    double breakForce=std::numeric_limits<double>::infinity();
    double lambda=0,penalty=1000;
    bool enabled=true;
};
struct Statistics {
    int pairs=0, manifolds=0, contacts=0;
    double maxPenetration=0, maxSpeed=0, maxAngularSpeed=0;
    int impactEvents=0, ccdSubsteps=1, brokenJoints=0, sleepingBodies=0;
    int ccdEvents=0, ccdCandidates=0, ccdUnresolved=0, ccdUnsupportedRotation=0, solverColors=0;
    int broadphaseCandidates=0, collisionIslands=0, largestIsland=0;
};
struct Settings {
    double dt=1.0/120.0;
    Vec3 gravity={0,-9.81,0};
    int iterations=24;
    int postIterations=20; // AVBD post-stabilization after BDF1 velocity reconstruction
    double beta=2000; // Dual penalty ramp rate
    double alpha=0.995; // Preexisting contact error stabilization
    double gamma=0.98; // Dual warmstarting decay
    double contactMargin=0.001;
    double initialPenalty=1000;
    double maxPenalty=1000000;
    double restitutionThreshold=0.5; // Ignore very slow contact restitution
    bool enableCCD=false; // Event-driven translation CCD for spheres and oriented boxes; rotating/accelerating TOI is approximate
    int maxCCDSteps=48;
    bool enableParallelSolver=false; // Independent graph-colored body updates (OpenMP if available)
    bool enableSpatialBroadphase=false; // Deterministic 3D BVH, optional alternative to 1D sweep-and-prune
    bool enableParallelNarrowphase=false; // Parallel independent contact generation; stable serial manifold merge
    bool enableIslandSolver=false; // Parallel disconnected constraint islands, with graph-color fallback
    int parallelThreads=0; // 0 = OpenMP default
    bool enableAdaptiveSubsteps=false; // Conservative discrete substepping; NOT exact swept CCD
    int maxSubsteps=64;
    double maxMotionFraction=0.3; // Movement <= fraction of minimum moving shape extent
    bool enableSleeping=false;
    double sleepLinearThreshold=0.06;
    double sleepAngularThreshold=0.08;
    double sleepAfterSeconds=0.65;
    double wakeLinearThreshold=0.2;
    // No artificially imposed velocity damping or contact force cap
};
class World {
public:
    Settings settings;
    int addBox(Vec3 center, Vec3 size,double density,double friction=0.6,Quat orientation={});
    int addSphere(Vec3 center,double radius,double density,double friction=0.6);
    int addDistanceJoint(int a,int b,Vec3 worldAnchorA,Vec3 worldAnchorB,
                         double restLength,double stiffness=std::numeric_limits<double>::infinity(),
                         double breakForce=std::numeric_limits<double>::infinity());
    const std::vector<DistanceJoint>& joints()const{return joints_;}
    Body& body(int id){return bodies_.at(static_cast<size_t>(id));}
    const Body& body(int id) const {return bodies_.at(static_cast<size_t>(id));}
    const std::vector<Body>& bodies() const {return bodies_;}
    const Statistics& statistics() const{return stats_;}
    void wakeBody(int id);
    void step();
private:
    void stepDiscrete();
    void stepCCD();
    std::vector<Body> bodies_;
    std::vector<DistanceJoint> joints_;
    std::map<std::pair<int,int>,Manifold> manifolds_;
    Statistics stats_;
};
}
