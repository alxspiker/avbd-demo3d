#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <set>
#include <numeric>
#include <functional>
#include <atomic>
#include <unordered_set>
#include <cstdint>
#ifdef AVBD_HAS_OPENMP
#include <omp.h>
#endif

namespace avbd {
static constexpr double EPS=1e-12;
double length(Vec3 v){return std::sqrt(length2(v));}
Vec3 normalized(Vec3 v){double d=length(v);return d>EPS?v/d:Vec3(0,1,0);}
Quat Quat::operator*(Quat b) const {return {w*b.w-x*b.x-y*b.y-z*b.z,w*b.x+x*b.w+y*b.z-z*b.y,w*b.y-x*b.z+y*b.w+z*b.x,w*b.z+x*b.y-y*b.x+z*b.w};}
Quat Quat::unit() const {double n=std::sqrt(w*w+x*x+y*y+z*z);return n>EPS?Quat(w/n,x/n,y/n,z/n):Quat{};}
Vec3 Quat::rotate(Vec3 v) const {Vec3 qv{x,y,z};return v+2.0*(w*cross(qv,v)+cross(qv,cross(qv,v)));}
Quat Quat::rotation(Vec3 axis,double radians){axis=normalized(axis);double a=radians*.5,s=std::sin(a);return {std::cos(a),axis.x*s,axis.y*s,axis.z*s};}
Quat Quat::exp(Vec3 rv){double a=length(rv);if(a<1e-10)return Quat(1,rv.x*.5,rv.y*.5,rv.z*.5).unit();return rotation(rv/a,a);}
Vec3 Quat::log() const {Quat q=unit();if(q.w<0){q={-q.w,-q.x,-q.y,-q.z};}double s=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z);if(s<1e-12)return Vec3{q.x,q.y,q.z}*2;double angle=2*std::atan2(s,q.w);return Vec3(q.x,q.y,q.z)*(angle/s);}

struct Mat3 {double a[3][3]{}; Vec3 times(Vec3 x) const{return {a[0][0]*x.x+a[0][1]*x.y+a[0][2]*x.z,a[1][0]*x.x+a[1][1]*x.y+a[1][2]*x.z,a[2][0]*x.x+a[2][1]*x.y+a[2][2]*x.z};}};
static Mat3 worldInertia(const Body& b){Mat3 I;Vec3 axes[3]={b.q.rotate({1,0,0}),b.q.rotate({0,1,0}),b.q.rotate({0,0,1})};for(int k=0;k<3;k++)for(int i=0;i<3;i++)for(int j=0;j<3;j++)I.a[i][j]+=b.inertia[k]*axes[k][i]*axes[k][j];return I;}
static Vec3 inverseInertiaTimes(const Body& b,Vec3 v){
    if(!b.dynamic())return {};
    const Vec3 axes[3]={b.q.rotate({1,0,0}),b.q.rotate({0,1,0}),b.q.rotate({0,0,1})};
    return axes[0]*(dot(axes[0],v)*b.invInertia.x)+axes[1]*(dot(axes[1],v)*b.invInertia.y)+axes[2]*(dot(axes[2],v)*b.invInertia.z);
}
static Vec3 contactVelocity(const Body& b,Vec3 localPoint){return b.velocity+cross(b.angularVelocity,b.q.rotate(localPoint));}
struct OBB {Vec3 c,h,axis[3];};
static OBB obb(const Body& b){return {b.p,b.half,{b.q.rotate({1,0,0}),b.q.rotate({0,1,0}),b.q.rotate({0,0,1})}};}
static double radius(const OBB& a,Vec3 n){return std::abs(dot(n,a.axis[0]))*a.h.x+std::abs(dot(n,a.axis[1]))*a.h.y+std::abs(dot(n,a.axis[2]))*a.h.z;}
static Vec3 local(const Body& b,Vec3 p){return b.q.conjugate().rotate(p-b.p);}
static Vec3 anchor(const Body& b,Vec3 localPoint){return b.p+b.q.rotate(localPoint);}
struct Axis {double sep=-1e100;Vec3 n;int type=-1,i=-1,j=-1;};
static bool satAxis(const OBB& a,const OBB& b,Vec3 delta,Vec3 v,int type,int i,int j,double margin,Axis& best){
    double l=length(v);if(l<1e-7)return true;Vec3 n=v/l;if(dot(n,delta)<0)n=-n;
    double sep=std::abs(dot(delta,n))-radius(a,n)-radius(b,n);
    if(sep>margin)return false;
    if(sep>best.sep) best={sep,n,type,i,j};
    return true;
}
static std::vector<Vec3> clip(const std::vector<Vec3>& input,Vec3 n,double offset){
    std::vector<Vec3> out;if(input.empty())return out;
    for(size_t i=0;i<input.size();i++){
        Vec3 a=input[i],b=input[(i+1)%input.size()];double da=dot(n,a)-offset,db=dot(n,b)-offset;
        bool ia=da<=1e-9,ib=db<=1e-9;
        if(ia)out.push_back(a);
        if(ia!=ib && std::abs(da-db)>EPS)out.push_back(a+(b-a)*(da/(da-db)));
    }
    return out;
}
static void appendContact(const Body& a,const Body& b,Vec3 pa,Vec3 pb,Vec3 normalBA,std::vector<Contact>& out){
    if(out.size()>=4)return;
    for(const auto& c:out){Vec3 mid=(anchor(a,c.rA)+anchor(b,c.rB))*.5;if(length2(mid-(pa+pb)*.5)<1e-8)return;}
    Contact c;c.rA=local(a,pa);c.rB=local(b,pb);c.n=normalBA;
    out.push_back(c);
}
static void faceContacts(const Body& A,const Body& B,const OBB& a,const OBB& b,const Axis& best,bool refIsA,std::vector<Contact>& result,double margin){
    const OBB& ref=refIsA?a:b;const OBB& inc=refIsA?b:a;
    const int axis=refIsA?best.i:best.j;
    Vec3 n=refIsA?best.n:-best.n; // reference face outward, toward incident body
    double sign=dot(ref.axis[axis],n)>=0?1:-1;
    Vec3 center=ref.c+ref.axis[axis]*(ref.h[axis]*sign);
    int u=(axis+1)%3,v=(axis+2)%3;
    int incAxis=0;double maxDot=-1;
    for(int j=0;j<3;j++){double d=std::abs(dot(inc.axis[j],n));if(d>maxDot){maxDot=d;incAxis=j;}}
    double incSign=dot(inc.axis[incAxis],n)>0?-1:1;
    Vec3 incCenter=inc.c+inc.axis[incAxis]*(inc.h[incAxis]*incSign);
    int iu=(incAxis+1)%3,iv=(incAxis+2)%3;
    Vec3 eu=inc.axis[iu]*inc.h[iu],ev=inc.axis[iv]*inc.h[iv];
    std::vector<Vec3> poly={incCenter+eu+ev,incCenter-eu+ev,incCenter-eu-ev,incCenter+eu-ev};
    for(int sideAxis:{u,v}){
        Vec3 dir=ref.axis[sideAxis];
        poly=clip(poly, dir, dot(dir,center)+ref.h[sideAxis]);
        poly=clip(poly,-dir, dot(-dir,center)+ref.h[sideAxis]);
    }
    std::vector<std::pair<double,Vec3>> points;
    for(Vec3 p:poly){double d=dot(p-center,n);if(d<=margin+1e-8)points.push_back({d,p});}
    std::sort(points.begin(),points.end(),[](auto& x,auto& y){return x.first<y.first;});
    // Deepest first; clip polygon supplies up to eight and four suffice for demonstration.
    for(auto& [d,p]:points){Vec3 projected=p-n*d;
        if(refIsA)appendContact(A,B,projected,p,-n,result);
        else appendContact(A,B,p,projected,n,result);
    }
}
static void closestSegments(Vec3 p1,Vec3 q1,Vec3 p2,Vec3 q2,Vec3& c1,Vec3& c2){
    Vec3 d1=q1-p1,d2=q2-p2,r=p1-p2;double a=dot(d1,d1),e=dot(d2,d2),f=dot(d2,r),s=0,t=0;
    if(a<EPS&&e<EPS){c1=p1;c2=p2;return;}
    if(a<EPS)t=std::clamp(f/e,0.0,1.0);
    else {double c=dot(d1,r);if(e<EPS)s=std::clamp(-c/a,0.0,1.0);
        else {double b=dot(d1,d2),denom=a*e-b*b;
            if(denom>EPS)s=std::clamp((b*f-c*e)/denom,0.0,1.0);
            t=(b*s+f)/e;
            if(t<0){t=0;s=std::clamp(-c/a,0.0,1.0);}else if(t>1){t=1;s=std::clamp((b-c)/a,0.0,1.0);}
        }
    }
    c1=p1+d1*s;c2=p2+d2*t;
}
static void edgeContacts(const Body& A,const Body& B,const OBB& a,const OBB& b,const Axis& best,std::vector<Contact>& result){
    int i=best.i,j=best.j;Vec3 ca=a.c,cb=b.c;
    for(int k=0;k<3;k++){if(k!=i)ca+=a.axis[k]*((dot(a.axis[k],best.n)>=0?1:-1)*a.h[k]);
                             if(k!=j)cb+=b.axis[k]*((dot(b.axis[k],-best.n)>=0?1:-1)*b.h[k]);}
    Vec3 pa,pb;closestSegments(ca-a.axis[i]*a.h[i],ca+a.axis[i]*a.h[i],cb-b.axis[j]*b.h[j],cb+b.axis[j]*b.h[j],pa,pb);
    appendContact(A,B,pa,pb,-best.n,result);
}
static bool sphereBox(const Body& s,const Body& b,double margin,Vec3& ps,Vec3& pb,Vec3& normal){
    const Vec3 bc=b.q.conjugate().rotate(s.p-b.p);
    Vec3 p={std::clamp(bc.x,-b.half.x,b.half.x),std::clamp(bc.y,-b.half.y,b.half.y),std::clamp(bc.z,-b.half.z,b.half.z)};
    Vec3 delta=bc-p;
    const double ds=length2(delta),r=s.half.x;
    if(ds>EPS){
        const double d=std::sqrt(ds);if(d>r+margin)return false;
        normal=b.q.rotate(delta/d);
    }else{
        // Sphere center inside box: push toward the nearest exit face.
        double best=1e100;int index=0;double sign=1;
        for(int i=0;i<3;i++)for(double si:{-1.0,1.0}){
            const double gap=b.half[i]-si*bc[i];
            if(gap<best){best=gap;index=i;sign=si;}
        }
        Vec3 n{}; if(index==0){n.x=sign;p.x=sign*b.half.x;}else if(index==1){n.y=sign;p.y=sign*b.half.y;}else{n.z=sign;p.z=sign*b.half.z;}
        normal=b.q.rotate(n);
    }
    pb=b.p+b.q.rotate(p);ps=s.p-normal*r;
    return true;
}
static std::vector<Contact> collide(const Body& A,const Body& B,double margin){
    if(A.shape==Shape::Sphere || B.shape==Shape::Sphere){
        std::vector<Contact> contacts;
        if(A.shape==Shape::Sphere && B.shape==Shape::Sphere){
            Vec3 d=A.p-B.p;const double dist=length(d),r=A.half.x+B.half.x;
            if(dist<=r+margin){Vec3 n=dist>EPS?d/dist:Vec3{1,0,0};appendContact(A,B,A.p-n*A.half.x,B.p+n*B.half.x,n,contacts);}
        }else if(A.shape==Shape::Sphere){
            Vec3 ps,pb,n;if(sphereBox(A,B,margin,ps,pb,n))appendContact(A,B,ps,pb,n,contacts);
        }else{
            Vec3 ps,pb,n;if(sphereBox(B,A,margin,ps,pb,n))appendContact(A,B,pb,ps,-n,contacts);
        }
        return contacts;
    }
    OBB a=obb(A),b=obb(B);Vec3 delta=b.c-a.c;Axis face,edge;
    for(int i=0;i<3;i++)if(!satAxis(a,b,delta,a.axis[i],0,i,-1,margin,face))return {};
    for(int j=0;j<3;j++)if(!satAxis(a,b,delta,b.axis[j],1,-1,j,margin,face))return {};
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)if(!satAxis(a,b,delta,cross(a.axis[i],b.axis[j]),2,i,j,margin,edge))return {};
    if(edge.type!=-1 && edge.sep>face.sep+0.015){std::vector<Contact> c;edgeContacts(A,B,a,b,edge,c);return c;}
    std::vector<Contact> c;faceContacts(A,B,a,b,face,face.type==0,c,margin);return c;
}
static Vec3 tangent1(Vec3 n){Vec3 t=std::abs(n.x)>0.65?cross(n,{0,1,0}):cross(n,{1,0,0});return normalized(t);}
static void addRow(double (&H)[6][6],double (&g)[6],const double* J,double k,double force){
    for(int i=0;i<6;i++){g[i]+=J[i]*force;for(int j=0;j<6;j++)H[i][j]+=k*J[i]*J[j];}
}
static bool solveSPD(double (&A)[6][6],double (&b)[6],double (&x)[6]){
    // Cholesky; the inertia + Gauss-Newton contact Hessian is SPD.
    double L[6][6]{};for(int i=0;i<6;i++){for(int j=0;j<=i;j++){
        double s=A[i][j];for(int k=0;k<j;k++)s-=L[i][k]*L[j][k];
        if(i==j){if(s<=1e-15||!std::isfinite(s))return false;L[i][j]=std::sqrt(s);}else L[i][j]=s/L[j][j];}}
    double y[6]{};for(int i=0;i<6;i++){double t=b[i];for(int j=0;j<i;j++)t-=L[i][j]*y[j];y[i]=t/L[i][i];}
    for(int i=5;i>=0;i--){double t=y[i];for(int j=i+1;j<6;j++)t-=L[j][i]*x[j];x[i]=t/L[i][i];}return true;
}
static std::array<double,6> jacobian(const Body& b,const Contact& c,Vec3 direction,bool isA){
    Vec3 r=b.q.rotate(isA?c.rA:c.rB);double sign=isA?1:-1;
    Vec3 lin=direction*sign,ang=cross(r,direction)*sign;
    return {lin.x,lin.y,lin.z,ang.x,ang.y,ang.z};
}
static double violation(const Body& a,const Body& b,const Contact& c,Vec3 dir,const Settings& settings,bool tangential,bool post){
    Vec3 delta=anchor(a,c.rA)-anchor(b,c.rB);
    if(tangential)return dot(delta-c.initialDelta,dir);
    double C0=dot(c.initialDelta,dir)+settings.contactMargin;
    return dot(delta-c.initialDelta,dir)+(post?C0:(1.0-settings.alpha)*C0);
}
int World::addBox(Vec3 center,Vec3 size,double density,double friction,Quat orientation){
    if(size.x<=0||size.y<=0||size.z<=0)throw std::invalid_argument("positive box extents required");
    if(density<0||friction<0)throw std::invalid_argument("density/friction must be nonnegative");
    Body b;b.id=static_cast<int>(bodies_.size());b.p=center;b.half=size*.5;b.q=orientation.unit();b.friction=friction;
    b.mass=density*size.x*size.y*size.z;b.invMass=b.mass>0?1/b.mass:0;
    if(b.dynamic()){
        b.inertia={b.mass*(size.y*size.y+size.z*size.z)/12.,b.mass*(size.x*size.x+size.z*size.z)/12.,b.mass*(size.x*size.x+size.y*size.y)/12.};
        b.invInertia={1/b.inertia.x,1/b.inertia.y,1/b.inertia.z};
    }
    bodies_.push_back(b);return b.id;
}
int World::addSphere(Vec3 center,double radius,double density,double friction){
    if(radius<=0 || !std::isfinite(radius))throw std::invalid_argument("positive finite sphere radius required");
    if(density<0||friction<0)throw std::invalid_argument("density/friction must be nonnegative");
    Body b;b.id=static_cast<int>(bodies_.size());b.shape=Shape::Sphere;b.p=center;
    b.half={radius,radius,radius};b.friction=friction;
    constexpr double pi=3.14159265358979323846;
    b.mass=density*(4.0/3.0)*pi*radius*radius*radius;b.invMass=b.mass>0?1/b.mass:0;
    if(b.dynamic()){
        const double i=0.4*b.mass*radius*radius;
        b.inertia={i,i,i};b.invInertia={1/i,1/i,1/i};
    }
    bodies_.push_back(b);return b.id;
}
int World::addDistanceJoint(int a,int b,Vec3 worldAnchorA,Vec3 worldAnchorB,
                            double restLength,double stiffness,double breakForce){
    if(a<0||b<0||a>=static_cast<int>(bodies_.size())||b>=static_cast<int>(bodies_.size())||a==b)
        throw std::invalid_argument("joint bodies must exist and be different");
    if(restLength<0||stiffness<=0||breakForce<=0)throw std::invalid_argument("invalid joint properties");
    DistanceJoint j;j.a=a;j.b=b;j.anchorA=local(bodies_[a],worldAnchorA);j.anchorB=local(bodies_[b],worldAnchorB);
    j.restLength=restLength;j.stiffness=stiffness;j.breakForce=breakForce;
    j.penalty=std::min(settings.initialPenalty,stiffness);
    joints_.push_back(j);return static_cast<int>(joints_.size()-1);
}
static double jointConstraint(const Body& a,const Body& b,const DistanceJoint& j,Vec3& n){
    const Vec3 d=anchor(a,j.anchorA)-anchor(b,j.anchorB);
    const double dist=length(d);
    n=dist>EPS?d/dist:Vec3{0,1,0};
    return dist-j.restLength;
}
struct Interval{int i;double minX,maxX;};
struct Bounds {Vec3 lo,hi;};
static bool boundsOverlap(const Bounds& a,const Bounds& b){
    return a.lo.x<=b.hi.x && b.lo.x<=a.hi.x &&
           a.lo.y<=b.hi.y && b.lo.y<=a.hi.y &&
           a.lo.z<=b.hi.z && b.lo.z<=a.hi.z;
}
static Bounds merged(Bounds a,Bounds b){
    return {{std::min(a.lo.x,b.lo.x),std::min(a.lo.y,b.lo.y),std::min(a.lo.z,b.lo.z)},
            {std::max(a.hi.x,b.hi.x),std::max(a.hi.y,b.hi.y),std::max(a.hi.z,b.hi.z)}};
}
// Rebuilt BVH: median-split spatial hierarchy. Unlike uniform grids it doesn't
// replicate large ground planes across thousands of cells. Leaves map to one body.
// Output is always sorted lexicographically, regardless of split/traversal order.
static std::vector<std::pair<int,int>> broadphase(const std::vector<Bounds>& boxes,bool useBVH){
    const int n=static_cast<int>(boxes.size());
    std::vector<std::pair<int,int>> pairs;
    if(!useBVH){
        std::vector<Interval> intervals;intervals.reserve(boxes.size());
        for(int i=0;i<n;i++)intervals.push_back({i,boxes[i].lo.x,boxes[i].hi.x});
        std::sort(intervals.begin(),intervals.end(),[](const Interval& a,const Interval& b){
            return a.minX==b.minX?a.i<b.i:a.minX<b.minX;
        });
        for(int i=0;i<n;i++)for(int j=i+1;j<n&&intervals[j].minX<=intervals[i].maxX;j++){
            int a=intervals[i].i,b=intervals[j].i;
            if(boundsOverlap(boxes[a],boxes[b]))pairs.emplace_back(std::min(a,b),std::max(a,b));
        }
    }else if(n>1){
        struct Node{Bounds box;int left=-1,right=-1,id=-1;};
        std::vector<Node> nodes;nodes.reserve(2*n);
        std::vector<int> order(n);std::iota(order.begin(),order.end(),0);
        std::function<int(int,int)> build=[&](int begin,int end)->int{
            const int index=static_cast<int>(nodes.size());nodes.push_back({});
            Bounds enclosing=boxes[order[begin]];
            for(int k=begin+1;k<end;k++)enclosing=merged(enclosing,boxes[order[k]]);
            nodes[index].box=enclosing;
            if(end-begin==1){nodes[index].id=order[begin];return index;}
            const Vec3 dimensions=enclosing.hi-enclosing.lo;
            const int axis=(dimensions.y>dimensions.x && dimensions.y>=dimensions.z)?1:
                            (dimensions.z>dimensions.x && dimensions.z>dimensions.y)?2:0;
            std::sort(order.begin()+begin,order.begin()+end,[&](int a,int b){
                const double ca=boxes[a].lo[axis]+boxes[a].hi[axis];
                const double cb=boxes[b].lo[axis]+boxes[b].hi[axis];
                return ca==cb?a<b:ca<cb;
            });
            int split=(begin+end)/2;
            const int l=build(begin,split),r=build(split,end);
            nodes[index].left=l;nodes[index].right=r;
            return index;
        };
        const int root=build(0,n);
        std::function<void(int,int)> visit=[&](int x,int y){
            const Node& a=nodes[x];const Node& b=nodes[y];
            if(!boundsOverlap(a.box,b.box))return;
            if(x==y){
                if(a.id>=0)return;
                visit(a.left,a.left);visit(a.left,a.right);visit(a.right,a.right);
            }else if(a.id>=0&&b.id>=0){
                pairs.emplace_back(std::min(a.id,b.id),std::max(a.id,b.id));
            }else if(a.id>=0 || (b.id<0 &&
                      length2(a.box.hi-a.box.lo)<length2(b.box.hi-b.box.lo))){
                visit(x,b.left);visit(x,b.right);
            }else{
                visit(a.left,y);visit(a.right,y);
            }
        };
        visit(root,root);
    }
    std::sort(pairs.begin(),pairs.end());
    return pairs;
}
// Stage 10: a *sufficient*, not necessary, proof that the predicted bodies
// cannot generate any discrete contacts. Each padded AABB is conservatively
// assigned to every 3D grid cell it touches. A cell used by two AABBs means
// we cannot certify the frame; run the normal solver without changing results.
// Large objects / static floors intentionally reject this fast path. This is
// a contact-free optimization, NOT swept CCD or a dense-contact GPU solver.
struct FreeCell {
    int64_t x,y,z;
    bool operator==(const FreeCell& b) const {return x==b.x && y==b.y && z==b.z;}
};
struct FreeCellHash {
    size_t operator()(const FreeCell& c)const{
        auto mix=[](uint64_t v){v^=v>>30;v*=0xbf58476d1ce4e5b9ULL;v^=v>>27;v*=0x94d049bb133111ebULL;return v^(v>>31);};
        return static_cast<size_t>(mix(static_cast<uint64_t>(c.x)) ^
            mix(static_cast<uint64_t>(c.y)+0x9e3779b97f4a7c15ULL) ^
            mix(static_cast<uint64_t>(c.z)+0x243f6a8885a308d3ULL));
    }
};
template <class InsertCell>
static bool certifyPredictedAABBs(const std::vector<Body>& bodies,const Settings& settings,InsertCell insert){
    const double cell=settings.freeFlightCellSize;
    if(!(cell>0) || !std::isfinite(cell))throw std::invalid_argument("freeFlightCellSize must be finite and positive");
    if(bodies.size()>static_cast<size_t>(std::numeric_limits<int>::max()))return false;
    const double dt=settings.dt;
    for(const Body& b:bodies){
        const Vec3 p=b.dynamic() ? b.p + b.velocity*dt + settings.gravity*(dt*dt) : b.p;
        const Quat q=b.dynamic() ? (Quat::exp(b.angularVelocity*dt)*b.q).unit():b.q;
        Vec3 r=b.half;
        if(b.shape==Shape::Box){
            const Vec3 ax[3]={q.rotate({1,0,0}),q.rotate({0,1,0}),q.rotate({0,0,1})};
            r={0,0,0};
            for(int j=0;j<3;j++)r+=Vec3{std::abs(ax[j].x),std::abs(ax[j].y),std::abs(ax[j].z)}*b.half[j];
        }
        const Vec3 pad{settings.contactMargin,settings.contactMargin,settings.contactMargin};
        const Vec3 shift{cell*.5,cell*.5,cell*.5};
        const Vec3 lo=(p-r-pad+shift)/cell,hi=(p+r+pad+shift)/cell;
        // Keep floor() in a range guaranteed safe for integer conversion.
        for(int j=0;j<3;j++)if(!std::isfinite(lo[j])||!std::isfinite(hi[j])||
            lo[j]<-1e12||hi[j]>1e12||hi[j]-lo[j]>1.0)return false;
        const int64_t ax=static_cast<int64_t>(std::floor(lo.x)),bx=static_cast<int64_t>(std::floor(hi.x));
        const int64_t ay=static_cast<int64_t>(std::floor(lo.y)),by=static_cast<int64_t>(std::floor(hi.y));
        const int64_t az=static_cast<int64_t>(std::floor(lo.z)),bz=static_cast<int64_t>(std::floor(hi.z));
        if(bx-ax>1||by-ay>1||bz-az>1)return false;
        for(int64_t x=ax;x<=bx;x++)for(int64_t y=ay;y<=by;y++)for(int64_t z=az;z<=bz;z++)
            if(!insert({x,y,z}))return false;
    }
    return true;
}
static bool certifySeparatedPredictedAABBs(const std::vector<Body>& bodies,const Settings& settings){
    std::unordered_set<FreeCell,FreeCellHash> occupied;
    occupied.reserve(bodies.size()*2);
    return certifyPredictedAABBs(bodies,settings,[&](FreeCell cell){
        return occupied.insert(cell).second;
    });
}
// A reusable, flat hash table avoids one heap allocation per occupied cell.
// If the table becomes excessively full, refuse the certificate and run
// ordinary contact detection: this affects performance, never correctness.
bool World::certifyFlatFreeFlight(){
    if(bodies_.size()>static_cast<size_t>(std::numeric_limits<int>::max()))return false;
    if(bodies_.size()>(std::numeric_limits<size_t>::max()/4))return false;
    size_t capacity=16;
    const size_t target=bodies_.size()*2+8;
    while(capacity<target){
        if(capacity>std::numeric_limits<size_t>::max()/2)return false;
        capacity*=2;
    }
    if(flatCellKeys_.size()<capacity){
        flatCellKeys_.resize(capacity);
        flatCellEpochs_.resize(capacity,0);
    }
    // Once grown, keep using the entire buffer so its hash mask stays correct.
    capacity=flatCellKeys_.size();
    if(++flatEpoch_==0){
        std::fill(flatCellEpochs_.begin(),flatCellEpochs_.end(),0);
        flatEpoch_=1;
    }
    const size_t mask=capacity-1;
    const FreeCellHash hash{};
    size_t filled=0;
    return certifyPredictedAABBs(bodies_,settings,[&](FreeCell c){
        if(filled>=capacity*7/10)return false;
        size_t slot=hash(c)&mask;
        for(size_t probe=0;probe<capacity;probe++){
            if(flatCellEpochs_[slot]!=flatEpoch_){
                flatCellEpochs_[slot]=flatEpoch_;
                flatCellKeys_[slot]={c.x,c.y,c.z};
                ++filled;
                return true;
            }
            const FlatCell& seen=flatCellKeys_[slot];
            if(seen.x==c.x&&seen.y==c.y&&seen.z==c.z)return false;
            slot=(slot+1)&mask;
        }
        return false;
    });
}

// Joint/contact connectivity through dynamic bodies only. A shared static
// floor must NOT merge two disconnected stacks into a single island.
static std::vector<std::vector<int>> buildIslands(const std::vector<Body>& bodies,
        const std::map<std::pair<int,int>,Manifold>& manifolds,
        const std::vector<DistanceJoint>& joints){
    const int n=static_cast<int>(bodies.size());
    std::vector<int> parent(n);std::iota(parent.begin(),parent.end(),0);
    auto root=[&](int i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    auto connect=[&](int a,int b){
        if(!bodies[a].dynamic()||bodies[a].sleeping||!bodies[b].dynamic()||bodies[b].sleeping)return;
        a=root(a);b=root(b);if(a!=b)parent[std::max(a,b)]=std::min(a,b);
    };
    for(const auto& [key,m]:manifolds)connect(m.a,m.b);
    for(const auto& j:joints)if(j.enabled)connect(j.a,j.b);
    std::map<int,std::vector<int>> groups;
    for(const Body& b:bodies)if(b.dynamic()&&!b.sleeping)groups[root(b.id)].push_back(b.id);
    std::vector<std::vector<int>> result;result.reserve(groups.size());
    for(auto& [key,group]:groups)result.push_back(std::move(group));
    return result;
}
void World::wakeBody(int id){Body& b=body(id);b.sleeping=false;b.quietTime=0;}
void World::step(){
    if(!settings.enableSleeping)for(auto& b:bodies_){b.sleeping=false;b.quietTime=0;}
    if(settings.enableSleeping)for(auto& b:bodies_)if(b.sleeping &&
        (length(b.velocity)>settings.sleepLinearThreshold || length(b.angularVelocity)>settings.sleepAngularThreshold))
        wakeBody(b.id);
    if(settings.dt<=0)throw std::invalid_argument("dt must be positive");
    if(settings.maxSubsteps<1||settings.maxMotionFraction<=0)throw std::invalid_argument("invalid adaptive substep settings");
    int substeps=1;
    if(settings.enableAdaptiveSubsteps){
        double maxRatio=0;
        for(const Body& b:bodies_)if(b.dynamic()){
            const double minSize=2*std::min({b.half.x,b.half.y,b.half.z});
            const double diameter=2*length(b.half);
            const double motion=(length(b.velocity)+length(b.angularVelocity)*diameter*.5+length(settings.gravity)*settings.dt)*settings.dt;
            maxRatio=std::max(maxRatio,motion/(settings.maxMotionFraction*minSize));
        }
        substeps=std::clamp(static_cast<int>(std::ceil(std::min(maxRatio,1e9))),1,settings.maxSubsteps);
    }
    if(settings.enableCCD){stepCCD();return;}
    if(substeps==1){stepDiscrete();stats_.ccdSubsteps=1;return;}
    const double fullDT=settings.dt;
    Statistics aggregate{};
    try{
        settings.dt=fullDT/substeps;
        for(int i=0;i<substeps;i++){
            stepDiscrete();
            aggregate.impactEvents+=stats_.impactEvents;
            aggregate.brokenJoints+=stats_.brokenJoints;
            aggregate.pairs+=stats_.pairs;
            aggregate.broadphaseCandidates+=stats_.broadphaseCandidates;
            aggregate.maxPenetration=std::max(aggregate.maxPenetration,stats_.maxPenetration);
            aggregate.maxSpeed=std::max(aggregate.maxSpeed,stats_.maxSpeed);
            aggregate.maxAngularSpeed=std::max(aggregate.maxAngularSpeed,stats_.maxAngularSpeed);
        }
    }catch(...){settings.dt=fullDT;throw;}
    settings.dt=fullDT;
    aggregate.manifolds=stats_.manifolds;aggregate.contacts=stats_.contacts;
    aggregate.sleepingBodies=stats_.sleepingBodies;aggregate.ccdSubsteps=substeps;
    aggregate.solverColors=stats_.solverColors;aggregate.collisionIslands=stats_.collisionIslands;aggregate.largestIsland=stats_.largestIsland;
    stats_=aggregate;
}
// Analytic swept intersections for linear translation with fixed orientation.
// Input velocities describe the step's chord; gravity and angular motion are
// not treated as exact continuous trajectories, so this is not general 6-DOF CCD.
static constexpr double NO_HIT=std::numeric_limits<double>::infinity();
static double sweepSphereSphere(const Body& a,const Body& b,Vec3 va,Vec3 vb,double dt,double margin){
    Vec3 d=a.p-b.p,v=va-vb;
    double rr=a.half.x+b.half.x+margin, c=dot(d,d)-rr*rr;
    if(c<=0)return NO_HIT; // Existing contact handled by discrete solver.
    double aa=dot(v,v),bb=2*dot(d,v),disc=bb*bb-4*aa*c;
    if(aa<1e-24||bb>=0||disc<0)return NO_HIT;
    double t=(-bb-std::sqrt(disc))/(2*aa);
    return t>=0&&t<=dt?t:NO_HIT;
}
static double boxPointDistance2(Vec3 p,Vec3 h){
    return length2({std::max(0.,std::abs(p.x)-h.x),std::max(0.,std::abs(p.y)-h.y),std::max(0.,std::abs(p.z)-h.z)});
}
static double sweepSphereBox(const Body& sphere,const Body& box,Vec3 vs,Vec3 vb,double dt,double margin){
    Vec3 p=box.q.conjugate().rotate(sphere.p-box.p);
    Vec3 v=box.q.conjugate().rotate(vs-vb);
    const double r=sphere.half.x+margin, rr=r*r;
    if(boxPointDistance2(p,box.half)<=rr)return NO_HIT;
    // The squared point-to-AABB distance is a quadratic on each interval
    // between crossings of the six axis-aligned face planes. Enumerating all
    // intervals yields the exact first intersection with the rounded box.
    std::vector<double> cuts{0,dt};
    for(int i=0;i<3;i++)if(std::abs(v[i])>1e-14)for(double sign:{-1.,1.}){
        double t=(sign*box.half[i]-p[i])/v[i];
        if(t>0&&t<dt)cuts.push_back(t);
    }
    std::sort(cuts.begin(),cuts.end());
    cuts.erase(std::unique(cuts.begin(),cuts.end(),[](double a,double b){return std::abs(a-b)<1e-12;}),cuts.end());
    for(size_t interval=1;interval<cuts.size();interval++){
        double lo=cuts[interval-1],hi=cuts[interval];
        Vec3 mid=p+v*((lo+hi)*0.5);
        double A=0,B=0,C=-rr;
        for(int i=0;i<3;i++){
            double edge=mid[i]<-box.half[i]?-box.half[i]:(mid[i]>box.half[i]?box.half[i]:mid[i]);
            if(mid[i]>=-box.half[i]&&mid[i]<=box.half[i])continue;
            const double offset=p[i]-edge;
            A+=v[i]*v[i];B+=2*v[i]*offset;C+=offset*offset;
        }
        if(A<1e-24){if(C<=0)return lo;continue;}
        const double disc=B*B-4*A*C;
        if(disc<0)continue;
        const double root=(-B-std::sqrt(std::max(0.,disc)))/(2*A);
        if(root>=lo-1e-10&&root<=hi+1e-10)return std::clamp(root,lo,hi);
    }
    return NO_HIT;
}
static double sweepBoxBox(const Body& a,const Body& b,Vec3 va,Vec3 vb,double dt,double margin){
    const OBB A=obb(a),B=obb(b);
    const Vec3 rel=vb-va,d=b.p-a.p;
    double enter=0,exit=dt;
    bool separated=false;
    auto axis=[&](Vec3 raw){
        const double l=length(raw);if(l<1e-8)return true;
        const Vec3 n=raw/l;
        const double extent=radius(A,n)+radius(B,n)+margin;
        const double dist=dot(d,n),speed=dot(rel,n);
        if(std::abs(dist)>extent)separated=true;
        if(std::abs(speed)<1e-14)return std::abs(dist)<=extent;
        double u=(-extent-dist)/speed,v=(extent-dist)/speed;
        if(u>v)std::swap(u,v);
        enter=std::max(enter,u);exit=std::min(exit,v);
        return enter<=exit;
    };
    for(int i=0;i<3;i++)if(!axis(A.axis[i]))return NO_HIT;
    for(int i=0;i<3;i++)if(!axis(B.axis[i]))return NO_HIT;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)if(!axis(cross(A.axis[i],B.axis[j])))return NO_HIT;
    return separated && enter>=0 && enter<=dt && enter<=exit ? enter:NO_HIT;
}
// Conservative advancement for constant linear/angular velocities. Unlike the
// exact translational sweeps, this uses a Lipschitz bound on separation and
// stops at the contact margin. It does not model angular acceleration.
static double rotationalSeparation(const Body& a,const Body& b){
    if(a.shape==Shape::Sphere&&b.shape==Shape::Sphere)
        return length(a.p-b.p)-a.half.x-b.half.x;
    if(a.shape==Shape::Sphere||b.shape==Shape::Sphere){
        const Body& sphere=a.shape==Shape::Sphere?a:b;
        const Body& box=a.shape==Shape::Box?a:b;
        const Vec3 localP=box.q.conjugate().rotate(sphere.p-box.p);
        return std::sqrt(boxPointDistance2(localP,box.half))-sphere.half.x;
    }
    const OBB A=obb(a),B=obb(b);
    const Vec3 d=B.c-A.c;
    double gap=-std::numeric_limits<double>::infinity();
    auto test=[&](Vec3 axis){
        double len=length(axis);
        if(len<1e-8)return;
        axis=axis/len;
        gap=std::max(gap,std::abs(dot(d,axis))-radius(A,axis)-radius(B,axis));
    };
    for(int i=0;i<3;i++){test(A.axis[i]);test(B.axis[i]);}
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)test(cross(A.axis[i],B.axis[j]));
    return gap;
}
static double sweepRotating(const Body& a,const Body& b,Vec3 va,Vec3 vb,double dt,double margin,bool& exhausted){
    exhausted=false;
    // Bounding radius times angular speed bounds each body's surface motion.
    const double ra=length(a.half),rb=length(b.half);
    const double speed=length(va-vb)+length(a.angularVelocity)*ra+length(b.angularVelocity)*rb;
    if(speed<1e-12)return NO_HIT;
    if(rotationalSeparation(a,b)<=margin)return NO_HIT;
    double t=0;
    for(int iteration=0;iteration<512;iteration++){
        Body A=a,B=b;
        A.p+=va*t;B.p+=vb*t;
        A.q=(Quat::exp(a.angularVelocity*t)*a.q).unit();
        B.q=(Quat::exp(b.angularVelocity*t)*b.q).unit();
        const double separation=rotationalSeparation(A,B);
        if(separation<=margin+1e-8)return t;
        // A conservative time step: no surface can close faster than speed.
        const double step=(separation-margin)/speed;
        if(step<1e-12)return t;
        t+=step;
        if(t>dt)return NO_HIT;
    }
    // Exhausting the conservative-advancement budget does NOT certify a miss.
    exhausted=true;
    return NO_HIT;
}
static Vec3 ccdVelocity(const Body& b,Vec3 gravity,double dt){
    return b.dynamic()&&!b.sleeping?b.velocity+gravity*dt:Vec3{};
}
void World::stepCCD(){
    if(settings.maxCCDSteps<1)throw std::invalid_argument("maxCCDSteps must be positive");
    const double fullDt=settings.dt;
    double remaining=fullDt;
    Statistics aggregate{};
    int count=0;
    try{
        while(remaining>fullDt*1e-10){
            if(count>=settings.maxCCDSteps){
                // A finite event budget cannot provide a no-tunnelling guarantee.
                // Report the unresolved window rather than silently claim CCD succeeded.
                ++aggregate.ccdUnresolved;
                settings.dt=remaining;stepDiscrete();
                ++count;break;
            }
            std::vector<Vec3> minPos(bodies_.size()),maxPos(bodies_.size()),vel(bodies_.size());
            std::vector<Bounds> sweptBounds(bodies_.size());
            for(const Body& b:bodies_){
                Vec3 ext;
                if(b.shape==Shape::Sphere)ext=b.half;
                else if(length2(b.angularVelocity)>1e-12){
                    const double r=length(b.half);ext={r,r,r};
                }else{
                    OBB o=obb(b);
                    for(int k=0;k<3;k++)ext+=Vec3{std::abs(o.axis[k].x),std::abs(o.axis[k].y),std::abs(o.axis[k].z)}*o.h[k];
                }
                Vec3 v=ccdVelocity(b,settings.gravity,remaining);
                vel[b.id]=v;
                Vec3 dest=b.p+v*remaining;
                Vec3 lo{std::min(b.p.x,dest.x),std::min(b.p.y,dest.y),std::min(b.p.z,dest.z)};
                Vec3 hi{std::max(b.p.x,dest.x),std::max(b.p.y,dest.y),std::max(b.p.z,dest.z)};
                minPos[b.id]=lo-ext;maxPos[b.id]=hi+ext;
                const Vec3 pad{settings.contactMargin,settings.contactMargin,settings.contactMargin};
                sweptBounds[b.id]={minPos[b.id]-pad,maxPos[b.id]+pad};
            }
            const auto candidatePairs=broadphase(sweptBounds,settings.enableSpatialBroadphase);
            aggregate.broadphaseCandidates+=static_cast<int>(candidatePairs.size());
            std::set<std::pair<int,int>> linked;
            for(const auto& j:joints_)if(j.enabled)linked.insert({std::min(j.a,j.b),std::max(j.a,j.b)});
            double first=NO_HIT;
            double firstSpeed=0;
            for(const auto& [ia,ib]:candidatePairs){
                const Body& a=bodies_[ia];const Body& b=bodies_[ib];
                if((!a.dynamic()&&!b.dynamic())||(a.sleeping&&b.sleeping))continue;
                if(linked.count({ia,ib}))continue;
                const bool rotating=(a.shape==Shape::Box&&length2(a.angularVelocity)>1e-12)||
                                    (b.shape==Shape::Box&&length2(b.angularVelocity)>1e-12);
                ++aggregate.ccdCandidates;
                double hit=NO_HIT;
                if(rotating){
                    bool exhausted=false;
                    hit=sweepRotating(a,b,vel[ia],vel[ib],remaining,settings.contactMargin,exhausted);
                    if(exhausted)++aggregate.ccdUnresolved;
                    // Finite iteration budgets and angular acceleration still
                    // preclude a universal no-tunnelling guarantee.
                }
                else if(a.shape==Shape::Sphere && b.shape==Shape::Sphere)hit=sweepSphereSphere(a,b,vel[ia],vel[ib],remaining,settings.contactMargin);
                else if(a.shape==Shape::Sphere)hit=sweepSphereBox(a,b,vel[ia],vel[ib],remaining,settings.contactMargin);
                else if(b.shape==Shape::Sphere)hit=sweepSphereBox(b,a,vel[ib],vel[ia],remaining,settings.contactMargin);
                else hit=sweepBoxBox(a,b,vel[ia],vel[ib],remaining,settings.contactMargin);
                if(hit<first){
                    first=hit;
                    firstSpeed=length(vel[ia]-vel[ib]);
                    if(rotating)firstSpeed+=length(a.angularVelocity)*length(a.half)+length(b.angularVelocity)*length(b.half);
                }
            }
            double advance=remaining;
            if(std::isfinite(first) && first<remaining){
                // Advance slightly past first contact, within the existing
                // narrowphase margin, so normal and restitution constraints activate.
                const double extra=std::max(1e-12,std::min(1e-5,settings.contactMargin*0.15/(firstSpeed+1e-9)));
                advance=std::min(remaining,std::max(first+extra,fullDt*1e-7));
                ++aggregate.ccdEvents;
            }
            settings.dt=advance;
            stepDiscrete();
            aggregate.impactEvents+=stats_.impactEvents;
            aggregate.brokenJoints+=stats_.brokenJoints;
            aggregate.pairs+=stats_.pairs;
            aggregate.broadphaseCandidates+=stats_.broadphaseCandidates;
            aggregate.maxPenetration=std::max(aggregate.maxPenetration,stats_.maxPenetration);
            aggregate.maxSpeed=std::max(aggregate.maxSpeed,stats_.maxSpeed);
            aggregate.maxAngularSpeed=std::max(aggregate.maxAngularSpeed,stats_.maxAngularSpeed);
            remaining-=advance;
            ++count;
        }
    }catch(...){settings.dt=fullDt;throw;}
    settings.dt=fullDt;
    aggregate.manifolds=stats_.manifolds;
    aggregate.contacts=stats_.contacts;
    aggregate.sleepingBodies=stats_.sleepingBodies;
    aggregate.ccdSubsteps=count;
    aggregate.solverColors=stats_.solverColors;
    aggregate.collisionIslands=stats_.collisionIslands;
    aggregate.largestIsland=stats_.largestIsland;
    stats_=aggregate;
}

// Stage 11: persistent structure-of-arrays buffers are fed from the public,
// authoritative AoS Body state on EVERY step. This is necessary because callers
// can retain and mutate Body& references. Each stream is contiguous and may be
// vectorized independently; quaternion dynamics remain in the original engine.
// The contact/constraint solver consumes the same Body values as before.
// Not a GPU solver, nor a replacement for dense-contact AoS constraint storage.
void World::predictDataOriented(){
    const size_t count=bodies_.size();
    motion_.resize(count); // reuses previously allocated storage
    const double dt=settings.dt;
    const Vec3 gravityStep=settings.gravity*(dt*dt);
    if(count>static_cast<size_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("data-oriented predictor exceeded supported body count");
    const int n=static_cast<int>(count);
    // Gather: body mutations made through World::body() remain observable.
#ifdef AVBD_HAS_OPENMP
#pragma omp parallel for schedule(static) if(n>=50000 && settings.parallelThreads>1) num_threads(settings.parallelThreads>0?settings.parallelThreads:1)
#endif
    for(int i=0;i<n;i++){
        const Body& b=bodies_[i];
        motion_.moving[i]=static_cast<uint8_t>(b.dynamic()&&!b.sleeping);
        motion_.px[i]=b.p.x;motion_.py[i]=b.p.y;motion_.pz[i]=b.p.z;
        motion_.vx[i]=b.velocity.x;motion_.vy[i]=b.velocity.y;motion_.vz[i]=b.velocity.z;
    }
    // Actual engine inertial translation update, not an isolated benchmark.
#ifdef AVBD_HAS_OPENMP
#pragma omp parallel for schedule(static) if(n>=50000 && settings.parallelThreads>1) num_threads(settings.parallelThreads>0?settings.parallelThreads:1)
#endif
    for(int i=0;i<n;i++)if(motion_.moving[i]){
        motion_.px[i]+=motion_.vx[i]*dt+gravityStep.x;
        motion_.py[i]+=motion_.vy[i]*dt+gravityStep.y;
        motion_.pz[i]+=motion_.vz[i]*dt+gravityStep.z;
    }
    // Scatter while preserving the existing full 3D rotational prediction.
#ifdef AVBD_HAS_OPENMP
#pragma omp parallel for schedule(static) if(n>=50000 && settings.parallelThreads>1) num_threads(settings.parallelThreads>0?settings.parallelThreads:1)
#endif
    for(int i=0;i<n;i++)if(motion_.moving[i]){
        Body& b=bodies_[i];
        b.p={motion_.px[i],motion_.py[i],motion_.pz[i]};
        b.q=(Quat::exp(b.angularVelocity*dt)*b.q).unit();
    }
}

void World::stepDiscrete(){
    const double dt=settings.dt;if(settings.postIterations<1||settings.iterations<1)throw std::runtime_error("positive solver iteration counts required");if(dt<=0)throw std::runtime_error("positive fixed step required");
    // A certified isolated discrete frame has no constraint rows. The unconstrained
    // 6x6 AVBD minimizer is exactly the inertial predictor, so iterating the
    // 6x6 solver would do no work. We still reconstruct velocities and validate
    // finite state. We never enter this path with previous manifolds or joints.
    if(settings.enableCertifiedFreeFlight && !settings.enableCCD &&
       !settings.enableSleeping && !settings.enableAdaptiveSubsteps &&
       joints_.empty() && manifolds_.empty() &&
       (settings.enableFlatFreeFlightCertificate ? certifyFlatFreeFlight() :
        certifySeparatedPredictedAABBs(bodies_,settings))){
        stats_={};stats_.certifiedFreeFlight=1;stats_.ccdSubsteps=1;
        // The packed predictor updates exactly the same engine bodies.
        // Preserve old positions/orientations for BDF1 reconstruction.
        std::vector<Vec3> previousP;
        std::vector<Quat> previousQ;
        if(settings.enableDataOrientedPredictor){
            previousP.reserve(bodies_.size());previousQ.reserve(bodies_.size());
            for(const Body& b:bodies_){previousP.push_back(b.p);previousQ.push_back(b.q);}
            predictDataOriented();
        }
        for(Body& b:bodies_)if(b.dynamic()){
            const Vec3 previous=settings.enableDataOrientedPredictor?previousP[b.id]:b.p;
            const Quat prevQ=settings.enableDataOrientedPredictor?previousQ[b.id]:b.q;
            if(!settings.enableDataOrientedPredictor){
                b.p+=b.velocity*dt+settings.gravity*(dt*dt);
                b.q=(Quat::exp(b.angularVelocity*dt)*b.q).unit();
            }
            b.velocity=(b.p-previous)/dt;
            b.angularVelocity=(b.q*prevQ.conjugate()).log()/dt;
            if(!std::isfinite(b.p.x)||!std::isfinite(b.p.y)||!std::isfinite(b.p.z)||!std::isfinite(b.q.w))
                throw std::runtime_error("non-finite rigid-body state");
            stats_.maxSpeed=std::max(stats_.maxSpeed,length(b.velocity));
            stats_.maxAngularSpeed=std::max(stats_.maxAngularSpeed,length(b.angularVelocity));
            ++stats_.freeFlightBodies;
        }
        stats_.collisionIslands=stats_.freeFlightBodies;
        stats_.largestIsland=stats_.freeFlightBodies>0?1:0;
        return;
    }
    stats_={};
    std::vector<Vec3> startP(bodies_.size()),startV(bodies_.size()),startW(bodies_.size());std::vector<Quat> startQ(bodies_.size());
    for(Body& b:bodies_){startP[b.id]=b.p;startQ[b.id]=b.q;startV[b.id]=b.velocity;startW[b.id]=b.angularVelocity;
        if(!settings.enableDataOrientedPredictor && b.dynamic()&&!b.sleeping){
            b.p+=b.velocity*dt+settings.gravity*(dt*dt);
            b.q=(Quat::exp(b.angularVelocity*dt)*b.q).unit();
        }
    }
    if(settings.enableDataOrientedPredictor)predictDataOriented();
    std::vector<Vec3> inertialP;std::vector<Quat> inertialQ;
    inertialP.reserve(bodies_.size());inertialQ.reserve(bodies_.size());
    for(const auto& b:bodies_){inertialP.push_back(b.p);inertialQ.push_back(b.q);}
    // Stable 3D BVH or the Stage 8 one-axis sweep, selected by settings.
    std::vector<Bounds> bounds(bodies_.size());
    for(const Body& b:bodies_){
        OBB ob=obb(b);Vec3 r;
        for(int i=0;i<3;i++)r+=Vec3{std::abs(ob.axis[i].x),std::abs(ob.axis[i].y),std::abs(ob.axis[i].z)}*ob.h[i];
        if(b.shape==Shape::Sphere)r=b.half;
        Vec3 pad{settings.contactMargin,settings.contactMargin,settings.contactMargin};
        bounds[b.id]={b.p-r-pad,b.p+r+pad};
    }
    const auto candidatePairs=broadphase(bounds,settings.enableSpatialBroadphase);
    stats_.broadphaseCandidates=static_cast<int>(candidatePairs.size());
    std::map<std::pair<int,int>,Manifold> next;
    // Adjacent joint links legitimately share collision volumes.
    std::set<std::pair<int,int>> linkedPairs;
    for(const auto& joint:joints_)if(joint.enabled)
        linkedPairs.insert({std::min(joint.a,joint.b),std::max(joint.a,joint.b)});
    std::vector<std::pair<int,int>> work;
    work.reserve(candidatePairs.size());
    for(const auto& [a,b]:candidatePairs){
        if(!bodies_[a].dynamic()&&!bodies_[b].dynamic())continue;
        if(linkedPairs.count({a,b}))continue;
        work.emplace_back(a,b);
    }
    stats_.pairs=static_cast<int>(work.size());
    // Each narrowphase call reads only body poses and writes a private slot.
    // Manifolds, warmstarting and contact IDs are merged in stable pair order.
    std::vector<std::vector<Contact>> generated(work.size());
    if(settings.parallelThreads<0)throw std::invalid_argument("parallelThreads must be >= 0");
    const int threads=settings.parallelThreads;
#ifdef AVBD_HAS_OPENMP
#pragma omp parallel for schedule(static) if(settings.enableParallelNarrowphase && work.size()>=64 && threads!=1) num_threads(threads>0?threads:omp_get_max_threads())
#endif
    for(int i=0;i<static_cast<int>(work.size());i++){
        auto [a,b]=work[i];
        generated[i]=collide(bodies_[a],bodies_[b],settings.contactMargin);
    }
    for(size_t i=0;i<work.size();i++){
        auto [a,b]=work[i];
        auto& contacts=generated[i];
        if(contacts.empty())continue;
        auto old=manifolds_.find({a,b});
        if(old!=manifolds_.end()){
            std::vector<bool> used(old->second.contacts.size());
            for(Contact& c:contacts){double score=1e99;int best=-1;
                for(size_t j=0;j<old->second.contacts.size();j++){
                    const auto& prev=old->second.contacts[j];
                    if(used[j]||dot(c.n,prev.n)<.97)continue;
                    double e=length2(anchor(bodies_[a],c.rA)-anchor(bodies_[a],prev.rA))+
                             length2(anchor(bodies_[b],c.rB)-anchor(bodies_[b],prev.rB));
                    if(e<score){score=e;best=static_cast<int>(j);}
                }
                if(best>=0&&score<0.06*0.06){const Contact& p=old->second.contacts[best];used[best]=true;
                    c.lambdaN=p.lambdaN*settings.gamma;c.lambdaT1=p.lambdaT1*settings.gamma;c.lambdaT2=p.lambdaT2*settings.gamma;
                    c.kN=p.kN*settings.gamma;c.kT1=p.kT1*settings.gamma;c.kT2=p.kT2*settings.gamma;c.matched=true;
                }
            }
        }
        for(Contact& c:contacts){c.kN=std::clamp(c.kN,settings.initialPenalty,settings.maxPenalty);
            c.kT1=std::clamp(c.kT1,settings.initialPenalty,settings.maxPenalty);
            c.kT2=std::clamp(c.kT2,settings.initialPenalty,settings.maxPenalty);
            c.initialDelta=(startP[a]+startQ[a].rotate(c.rA))-(startP[b]+startQ[b].rotate(c.rB));
            if(!c.matched){
                Vec3 rA=startQ[a].rotate(c.rA),rB=startQ[b].rotate(c.rB);
                Vec3 vA=startV[a]+cross(startW[a],rA),vB=startV[b]+cross(startW[b],rB);
                c.impactSpeed=std::min(0.0,dot(vA-vB,c.n));
            }
        }
        next[{a,b}]={a,b,std::move(contacts)};
    }
    manifolds_=std::move(next);stats_.manifolds=static_cast<int>(manifolds_.size());
    // Per-body force lists, rebuilt from stable manifold keys.
    std::vector<bool> touched(bodies_.size(),false);
    // Wake a resting body that is struck by a moving one. The whole contact
    // manifold will then participate in this step's AVBD primal iterations.
    if(settings.enableSleeping){
        for(const auto& [key,m]:manifolds_){
            Body& a=bodies_[m.a],&b=bodies_[m.b];
            if(a.sleeping && b.dynamic() && !b.sleeping &&
                 (length(b.velocity)>settings.wakeLinearThreshold||length(b.angularVelocity)>settings.wakeLinearThreshold))wakeBody(a.id);
            if(b.sleeping && a.dynamic() && !a.sleeping &&
                 (length(a.velocity)>settings.wakeLinearThreshold||length(a.angularVelocity)>settings.wakeLinearThreshold))wakeBody(b.id);
        }
        for(auto& j:joints_)if(j.enabled){
            // Conservative policy: joints never sleep; a fixed body is unaffected.
            if(bodies_[j.a].dynamic())wakeBody(j.a);
            if(bodies_[j.b].dynamic())wakeBody(j.b);
        }
    }
    std::vector<std::vector<Manifold*>> acting(bodies_.size());
    std::vector<std::vector<DistanceJoint*>> actingJoints(bodies_.size());
    for(auto& j:joints_)if(j.enabled){
        j.lambda*=settings.gamma;
        j.penalty=std::max(settings.initialPenalty,std::min(j.penalty*settings.gamma,std::min(j.stiffness,settings.maxPenalty)));
        actingJoints[j.a].push_back(&j);actingJoints[j.b].push_back(&j);
    }
    for(auto& [key,m]:manifolds_){acting[m.a].push_back(&m);acting[m.b].push_back(&m);
        touched[m.a]=touched[m.b]=true;
        stats_.contacts+=static_cast<int>(m.contacts.size());}
    auto islands=buildIslands(bodies_,manifolds_,joints_);
    stats_.collisionIslands=static_cast<int>(islands.size());
    for(const auto& island:islands)stats_.largestIsland=std::max(stats_.largestIsland,static_cast<int>(island.size()));
    // Parallel islands preserve the local serial update order, and can run
    // independently because static bodies are never mutable in the solver.
    const bool useIslandTasks=settings.enableParallelSolver && settings.enableIslandSolver && islands.size()>1;
    // Greedy contact-graph coloring. Every mutable body in the same color is
    // independent: its contacts/joints only reference other colors or statics.
    // Each color is a Gauss-Seidel wavefront; bodies within a color are parallel.
    std::vector<std::vector<int>> colors;
    if(settings.enableParallelSolver && !useIslandTasks){
        std::vector<std::vector<int>> neighbors(bodies_.size());
        auto connect=[&](int ia,int ib){
            if(bodies_[ia].dynamic()&&!bodies_[ia].sleeping && bodies_[ib].dynamic()&&!bodies_[ib].sleeping){
                neighbors[ia].push_back(ib);neighbors[ib].push_back(ia);
            }
        };
        for(const auto& [key,m]:manifolds_)connect(m.a,m.b);
        for(const auto& j:joints_)if(j.enabled)connect(j.a,j.b);
        std::vector<int> assigned(bodies_.size(),-1);
        for(const auto& b:bodies_)if(b.dynamic()&&!b.sleeping){
            int c=0;
            for(;;c++){
                bool conflict=false;
                for(int other:neighbors[b.id])if(assigned[other]==c){conflict=true;break;}
                if(!conflict)break;
            }
            if(c==static_cast<int>(colors.size()))colors.emplace_back();
            colors[c].push_back(b.id);assigned[b.id]=c;
        }
    }
    stats_.solverColors=static_cast<int>(colors.size());
    const int parallelThreads=settings.parallelThreads;
    if(parallelThreads<0)throw std::invalid_argument("parallelThreads must be >= 0");
    std::atomic<bool> solverFailure{false};
    // Only the owner body is modified. Contact and joint state is read-only
    // while the primal solve executes; dual updates occur after each wavefront.
    auto solveBody=[&](Body& body,bool post){
            double H[6][6]{},g[6]{};
            Mat3 I=worldInertia(body);
            double m=body.mass/(dt*dt);
            for(int i=0;i<3;i++)H[i][i]+=m;
            for(int i=0;i<3;i++)for(int j=0;j<3;j++)H[3+i][3+j]+=I.a[i][j]/(dt*dt);
            Vec3 linear=(body.p-inertialP[body.id])*m;
            Vec3 angular=I.times((body.q*inertialQ[body.id].conjugate()).log())/(dt*dt);
            g[0]=linear.x;g[1]=linear.y;g[2]=linear.z;g[3]=angular.x;g[4]=angular.y;g[5]=angular.z;
            for(Manifold* manifold:acting[body.id]){
                const Body& a=bodies_[manifold->a];const Body& b=bodies_[manifold->b];bool isA=body.id==a.id;
                double mu=std::sqrt(a.friction*b.friction);
                for(Contact& c:manifold->contacts){
                    Vec3 t1=tangent1(c.n),t2=cross(c.n,t1);
                    double cn=violation(a,b,c,c.n,settings,false,post);
                    // AVBD Eq. 13: clamped penalty + dual multipliers; inactive rows contribute no Hessian.
                    double kNormal=post?std::max(c.kN,50000.0):c.kN;
                    double fn=std::min(0.0,kNormal*cn+c.lambdaN);
                    if(fn<0){auto J=jacobian(body,c,c.n,isA);addRow(H,g,J.data(),kNormal,fn);}
                    double maxTangential=mu*std::max(-fn,-c.lambdaN);
                    // Approximate Coulomb cone by projecting combined tangential trial onto the disc.
                    double ct1=violation(a,b,c,t1,settings,true,post),ct2=violation(a,b,c,t2,settings,true,post);
                    double f1=c.kT1*ct1+c.lambdaT1,f2=c.kT2*ct2+c.lambdaT2;
                    double mag=std::hypot(f1,f2);if(mag>maxTangential&&mag>EPS){f1*=maxTangential/mag;f2*=maxTangential/mag;}
                    if(maxTangential>0){auto J1=jacobian(body,c,t1,isA),J2=jacobian(body,c,t2,isA);
                        addRow(H,g,J1.data(),c.kT1,f1);
                        addRow(H,g,J2.data(),c.kT2,f2);
                    }
                }
            }
            for(DistanceJoint* joint:actingJoints[body.id]){
                if(!joint->enabled)continue;
                Body& a=bodies_[joint->a];Body& b=bodies_[joint->b];
                Vec3 n;const double C=jointConstraint(a,b,*joint,n);
                const bool isA=(body.id==joint->a);
                const Vec3 arm=body.q.rotate(isA?joint->anchorA:joint->anchorB);
                const double sign=isA?1:-1;
                const Vec3 ang=cross(arm,n)*sign,lin=n*sign;
                const double J[6]={lin.x,lin.y,lin.z,ang.x,ang.y,ang.z};
                const double penalty=post?std::min(joint->stiffness,std::max(joint->penalty,50000.)):joint->penalty;
                const double force=std::clamp(joint->lambda+penalty*C,-joint->breakForce,joint->breakForce);
                addRow(H,g,J,penalty,force);
            }
            double dx[6]{};if(!solveSPD(H,g,dx)){solverFailure.store(true,std::memory_order_relaxed);return;}
            body.p-=Vec3(dx[0],dx[1],dx[2]);
            body.q=(Quat::exp(-Vec3(dx[3],dx[4],dx[5]))*body.q).unit();

    };
    for(int it=0;it<settings.iterations+settings.postIterations;it++){
        bool post=it>=settings.iterations;
        if(it==settings.iterations){
            // As in the 2D reference: BDF1 velocity is reconstructed before post-stabilization.
            // Geometric penetration repair does not become spurious kinetic energy.
            for(Body& b:bodies_)if(b.dynamic()&&!b.sleeping){
                b.velocity=(b.p-startP[b.id])/dt;
                b.angularVelocity=(b.q*startQ[b.id].conjugate()).log()/dt;
            }
        }
        if(useIslandTasks){
#ifdef AVBD_HAS_OPENMP
#pragma omp parallel for schedule(dynamic,16) if(islands.size()>1 && parallelThreads!=1) num_threads(parallelThreads>0?parallelThreads:omp_get_max_threads())
#endif
            for(int k=0;k<static_cast<int>(islands.size());k++)
                for(int id:islands[k])solveBody(bodies_[id],post);
        }else if(settings.enableParallelSolver){
            for(const auto& wave:colors){
#ifdef AVBD_HAS_OPENMP
#pragma omp parallel for schedule(static) if(wave.size()>=24 && parallelThreads!=1) num_threads(parallelThreads>0?parallelThreads:omp_get_max_threads())
#endif
                for(int k=0;k<static_cast<int>(wave.size());k++)
                    solveBody(bodies_[wave[k]],post);
            }
        }else{
            for(Body& b:bodies_)if(b.dynamic()&&!b.sleeping)solveBody(b,post);
        }
        if(solverFailure.load(std::memory_order_relaxed))throw std::runtime_error("AVBD 6x6 matrix lost positive definiteness");

        if(!post)for(auto& joint:joints_)if(joint.enabled){
            Vec3 n;const double C=jointConstraint(bodies_[joint.a],bodies_[joint.b],joint,n);
            const double desired=joint.lambda+joint.penalty*C;
            if(std::abs(desired)>=joint.breakForce){joint.enabled=false;stats_.brokenJoints++;continue;}
            joint.lambda=desired;
            joint.penalty=std::min({settings.maxPenalty,joint.stiffness,joint.penalty+settings.beta*std::abs(C)});
        }
        // Only the pre-stabilization phase updates duals and penalty state.
        if(!post)for(auto& [key,m]:manifolds_){const Body& a=bodies_[m.a],&b=bodies_[m.b];double mu=std::sqrt(a.friction*b.friction);
            for(Contact& c:m.contacts){
                double cn=violation(a,b,c,c.n,settings,false,post);
                c.lambdaN=std::min(0.0,c.lambdaN+c.kN*cn);
                if(c.lambdaN<0)c.kN=std::min(settings.maxPenalty,c.kN+settings.beta*std::abs(cn));
                Vec3 t1=tangent1(c.n),t2=cross(c.n,t1);
                double lt1=c.lambdaT1+c.kT1*violation(a,b,c,t1,settings,true,post);
                double lt2=c.lambdaT2+c.kT2*violation(a,b,c,t2,settings,true,post);
                double lim=mu*-c.lambdaN,mag=std::hypot(lt1,lt2);
                if(mag>lim&&mag>EPS){lt1*=lim/mag;lt2*=lim/mag;}
                c.lambdaT1=lt1;c.lambdaT2=lt2;
                c.kT1=std::min(settings.maxPenalty,c.kT1+settings.beta*std::abs(violation(a,b,c,t1,settings,true,post)));
                c.kT2=std::min(settings.maxPenalty,c.kT2+settings.beta*std::abs(violation(a,b,c,t2,settings,true,post)));
            }
        }
    }
    // Restitution is a velocity-level boundary condition after AVBD's non-bouncy
    // position solve. Only brand-new, closing contacts bounce. Apply sequential
    // impulses at their true world-space witness points, including angular mass.
    // This is an explicit hybrid extension; it is not part of the original AVBD demo.
    for(const auto& [key,m]:manifolds_){
        Body& a=bodies_[m.a];Body& b=bodies_[m.b];
        const double e=std::max(a.restitution,b.restitution);
        if(e<=0)continue;
        for(const Contact& c:m.contacts){
            if(c.matched || c.impactSpeed>=-settings.restitutionThreshold)continue;
            const Vec3 ra=a.q.rotate(c.rA),rb=b.q.rotate(c.rB);
            const double vn=dot(contactVelocity(a,c.rA)-contactVelocity(b,c.rB),c.n);
            const double target=-e*c.impactSpeed;
            if(vn>=target)continue;
            const Vec3 ja=cross(ra,c.n),jb=cross(rb,c.n);
            const double k=a.invMass+b.invMass+dot(ja,inverseInertiaTimes(a,ja))+dot(jb,inverseInertiaTimes(b,jb));
            if(k<1e-12)continue;
            const double impulse=(target-vn)/k;
            if(a.dynamic()){a.velocity+=c.n*(impulse*a.invMass);a.angularVelocity+=inverseInertiaTimes(a,ja)*impulse;}
            if(b.dynamic()){b.velocity-=c.n*(impulse*b.invMass);b.angularVelocity-=inverseInertiaTimes(b,jb)*impulse;}
            ++stats_.impactEvents;
        }
    }
    // BDF1 velocity reconstruction was performed before post-stabilization.
    for(Body& b:bodies_){if(!b.dynamic())continue;
        if(settings.enableSleeping){
            if(b.sleeping && !touched[b.id])wakeBody(b.id); // support disappeared
            else if(!b.sleeping){
                if(touched[b.id] && length(b.velocity)<settings.sleepLinearThreshold &&
                   length(b.angularVelocity)<settings.sleepAngularThreshold){
                    b.quietTime+=dt;
                    if(b.quietTime>=settings.sleepAfterSeconds){b.sleeping=true;b.velocity={};b.angularVelocity={};}
                }else b.quietTime=0;
            }
            if(b.sleeping)stats_.sleepingBodies++;
        }
        if(!std::isfinite(b.p.x)||!std::isfinite(b.p.y)||!std::isfinite(b.p.z)||!std::isfinite(b.q.w))throw std::runtime_error("non-finite rigid-body state");
        stats_.maxSpeed=std::max(stats_.maxSpeed,length(b.velocity));
        stats_.maxAngularSpeed=std::max(stats_.maxAngularSpeed,length(b.angularVelocity));
    }
    for(const auto& [key,m]:manifolds_)for(const Contact& c:m.contacts){const auto& a=bodies_[m.a];const auto& b=bodies_[m.b];
        stats_.maxPenetration=std::max(stats_.maxPenetration,std::max(0.0,-dot(anchor(a,c.rA)-anchor(b,c.rB),c.n)));
    }
}
} // namespace avbd
