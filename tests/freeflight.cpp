#include "avbd3d.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
using namespace avbd;
static void check(bool value,const char* msg){if(!value)throw std::runtime_error(msg);}
static World makeFree(int n){
    World w;w.settings.iterations=5;w.settings.postIterations=3;
    w.settings.gravity={0,-0.31,0};
    for(int i=0;i<n;i++){
        const int x=i%20,y=i/20;
        const int id=w.addBox({(x-10)*4.,15+y*4.,1.25*(i%3)},{.5,.5,.5},1,.5,
            Quat::rotation({0,1,0},.09*(i%5)));
        w.body(id).velocity={.04*(i%4),-.012,.005*(i%3)};
        w.body(id).angularVelocity={0,.06*(i%4),0};
    }
    return w;
}
static void same(const World& a,const World& b,double tol){
    check(a.bodies().size()==b.bodies().size(),"different body count");
    for(size_t j=0;j<a.bodies().size();j++){
        const auto& x=a.body(int(j));const auto& y=b.body(int(j));
        check(length(x.p-y.p)<tol,"free-flight position parity");
        check(length(x.velocity-y.velocity)<tol,"free-flight velocity parity");
        check(length(x.angularVelocity-y.angularVelocity)<tol,"free-flight angular velocity parity");
        check(std::abs(x.q.w-y.q.w)+std::abs(x.q.x-y.q.x)+std::abs(x.q.y-y.q.y)+std::abs(x.q.z-y.q.z)<tol,"free-flight rotation parity");
    }
}
int main(){try{
    World reference=makeFree(180),fast=reference;
    reference.settings.enableSpatialBroadphase=true;
    fast.settings.enableCertifiedFreeFlight=true;
    for(int i=0;i<8;i++){
        reference.step();fast.step();
        check(fast.statistics().certifiedFreeFlight==1,"unconstrained isolated bodies should be certified");
        check(fast.statistics().freeFlightBodies==180,"fast path body count");
        check(fast.statistics().contacts==0,"certified no-contact frame had contacts");
        same(reference,fast,1e-8);
    }
    std::puts("180-box rotating and translating worlds, eight-step legacy parity: PASS");
    {
        World w;w.settings.enableCertifiedFreeFlight=true;
        int a=w.addSphere({0,1,0},.5,1),b=w.addSphere({0,1.8,0},.5,1);
        w.body(a).velocity={0,0,0};w.body(b).velocity={0,0,0};
        w.step();check(!w.statistics().certifiedFreeFlight,"overlapping spheres were wrongly certified");
        check(w.statistics().contacts>0,"overlap did not reach contact solver");
    }
    std::puts("Nearby overlapping bodies reject certificate and keep collision response: PASS");
    {
        World w;w.settings.enableCertifiedFreeFlight=true;
        w.addBox({0,-.5,0},{10,1,10},0);
        w.addBox({0,.5,0},{1,1,1},1);
        w.step();check(!w.statistics().certifiedFreeFlight,"static floor should reject certificate");
        check(w.statistics().contacts>0,"ground contact lost");
    }
    std::puts("Large floor + small body rejects fast path and preserves contact: PASS");
    {
        World w;w.settings.enableCertifiedFreeFlight=true;
        int a=w.addSphere({0,2,0},.2,1),b=w.addSphere({4,2,0},.2,1);
        w.addDistanceJoint(a,b,{0,2,0},{4,2,0},4);
        w.step();check(!w.statistics().certifiedFreeFlight,"joint path was skipped");
    }
    std::puts("Joint presence prohibits contact-free shortcut: PASS");
    {
        World w=makeFree(4);w.settings.enableCertifiedFreeFlight=true;w.settings.enableCCD=true;
        w.step();check(!w.statistics().certifiedFreeFlight,"CCD path must not be bypassed");
    }
    {
        World w=makeFree(4);w.settings.enableCertifiedFreeFlight=true;w.settings.enableSleeping=true;
        w.step();check(!w.statistics().certifiedFreeFlight,"sleep path must not be bypassed");
    }
    {
        World w=makeFree(4);w.settings.enableCertifiedFreeFlight=true;w.settings.freeFlightCellSize=0;
        bool rejected=false;try{w.step();}catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"bad grid width should be rejected");
    }
    std::puts("CCD/sleeping guarded, invalid configuration rejected: PASS");
    std::puts("Stage 10 CPU certified sparse fast path: PASS");
    return 0;
}catch(const std::exception& e){std::fprintf(stderr,"Stage 10 FAIL: %s\n",e.what());return 1;}}
