#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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
static std::vector<Contact> collide(const Body& A,const Body& B,double margin){
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
struct Interval{int i;double minX,maxX;};
void World::step(){
    const double dt=settings.dt;if(settings.postIterations<1||settings.iterations<1)throw std::runtime_error("positive solver iteration counts required");if(dt<=0)throw std::runtime_error("positive fixed step required");
    stats_={};
    std::vector<Vec3> startP(bodies_.size());std::vector<Quat> startQ(bodies_.size());
    for(Body& b:bodies_){startP[b.id]=b.p;startQ[b.id]=b.q;
        if(b.dynamic()){
            b.p+=b.velocity*dt+settings.gravity*(dt*dt);
            b.q=(Quat::exp(b.angularVelocity*dt)*b.q).unit();
        }
    }
    std::vector<Vec3> inertialP;std::vector<Quat> inertialQ;
    inertialP.reserve(bodies_.size());inertialQ.reserve(bodies_.size());
    for(const auto& b:bodies_){inertialP.push_back(b.p);inertialQ.push_back(b.q);}
    // Sweep and prune broadphase on conservative rotated AABBs.
    std::vector<Interval> interval;interval.reserve(bodies_.size());
    std::vector<Vec3> aabb(bodies_.size());
    for(const Body& b:bodies_){OBB ob=obb(b);Vec3 r;
        for(int i=0;i<3;i++){r+=Vec3{std::abs(ob.axis[i].x),std::abs(ob.axis[i].y),std::abs(ob.axis[i].z)}*ob.h[i];}
        aabb[b.id]=r;interval.push_back({b.id,b.p.x-r.x-settings.contactMargin,b.p.x+r.x+settings.contactMargin});
    }
    std::sort(interval.begin(),interval.end(),[](const auto& a,const auto& b){return a.minX<b.minX;});
    std::map<std::pair<int,int>,Manifold> next;
    for(size_t i=0;i<interval.size();i++)for(size_t j=i+1;j<interval.size()&&interval[j].minX<=interval[i].maxX;j++){
        int a=std::min(interval[i].i,interval[j].i),b=std::max(interval[i].i,interval[j].i);
        if(!bodies_[a].dynamic()&&!bodies_[b].dynamic())continue;
        Vec3 d=bodies_[a].p-bodies_[b].p,r=aabb[a]+aabb[b];
        if(std::abs(d.y)>r.y+settings.contactMargin||std::abs(d.z)>r.z+settings.contactMargin)continue;
        stats_.pairs++;
        auto contacts=collide(bodies_[a],bodies_[b],settings.contactMargin);
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
        }
        next[{a,b}]={a,b,std::move(contacts)};
    }
    manifolds_=std::move(next);stats_.manifolds=static_cast<int>(manifolds_.size());
    // Per-body force lists, rebuilt from stable manifold keys.
    std::vector<std::vector<Manifold*>> acting(bodies_.size());
    for(auto& [key,m]:manifolds_){acting[m.a].push_back(&m);acting[m.b].push_back(&m);stats_.contacts+=static_cast<int>(m.contacts.size());}
    for(int it=0;it<settings.iterations+settings.postIterations;it++){
        bool post=it>=settings.iterations;
        if(it==settings.iterations){
            // As in the 2D reference: BDF1 velocity is reconstructed before post-stabilization.
            // Geometric penetration repair does not become spurious kinetic energy.
            for(Body& b:bodies_)if(b.dynamic()){
                b.velocity=(b.p-startP[b.id])/dt;
                b.angularVelocity=(b.q*startQ[b.id].conjugate()).log()/dt;
            }
        }
        for(Body& body:bodies_){if(!body.dynamic())continue;
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
                        addRow(H,g,*reinterpret_cast<const double(*)[6]>(J1.data()),c.kT1,f1);
                        addRow(H,g,*reinterpret_cast<const double(*)[6]>(J2.data()),c.kT2,f2);
                    }
                }
            }
            double dx[6]{};if(!solveSPD(H,g,dx))throw std::runtime_error("AVBD 6x6 matrix lost positive definiteness");
            body.p-=Vec3(dx[0],dx[1],dx[2]);
            body.q=(Quat::exp(-Vec3(dx[3],dx[4],dx[5]))*body.q).unit();
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
    // BDF1 velocity reconstruction was performed before post-stabilization.
    for(Body& b:bodies_){if(!b.dynamic())continue;
        if(!std::isfinite(b.p.x)||!std::isfinite(b.p.y)||!std::isfinite(b.p.z)||!std::isfinite(b.q.w))throw std::runtime_error("non-finite rigid-body state");
        stats_.maxSpeed=std::max(stats_.maxSpeed,length(b.velocity));
        stats_.maxAngularSpeed=std::max(stats_.maxAngularSpeed,length(b.angularVelocity));
    }
    for(const auto& [key,m]:manifolds_)for(const Contact& c:m.contacts){const auto& a=bodies_[m.a];const auto& b=bodies_[m.b];
        stats_.maxPenetration=std::max(stats_.maxPenetration,std::max(0.0,-dot(anchor(a,c.rA)-anchor(b,c.rB),c.n)));
    }
}
} // namespace avbd
