#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
using namespace avbd;
static void require(bool condition,const char*message){if(!condition)throw std::runtime_error(message);}
static double compare(const World& a,const World& b){
    require(a.bodies().size()==b.bodies().size(),"different body counts");
    double worst=0;
    for(size_t i=0;i<a.bodies().size();i++){
        const Body& x=a.body(static_cast<int>(i)),&y=b.body(static_cast<int>(i));
        worst=std::max({worst,length(x.p-y.p),length(x.velocity-y.velocity),
            length(x.angularVelocity-y.angularVelocity),std::abs(x.q.w-y.q.w),
            std::abs(x.q.x-y.q.x),std::abs(x.q.y-y.q.y),std::abs(x.q.z-y.q.z)});
        require(x.sleeping==y.sleeping,"sleep state diverged");
    }
    require(a.statistics().contacts==b.statistics().contacts,"contact count divergence");
    require(a.statistics().impactEvents==b.statistics().impactEvents,"impact count divergence");
    return worst;
}
int main(){try{
    // Case A: 512 independently moving rotated boxes; real World::step freeflight.
    World a;
    a.settings.enableCertifiedFreeFlight=true;
    a.settings.gravity={.17,-.31,.06};
    for(int i=0;i<512;i++){
        int id=a.addBox({double(i%32)*4,30+double(i/32)*4,0},{.5,.5,.5},1,.6,
            Quat::rotation({1,.3,.5},.013*(i%11)));
        a.body(id).velocity={double(i%7)*.02,-.05,double(i%9)*.01};
        a.body(id).angularVelocity={.01*(i%3),.03*(i%5),.005*(i%7)};
    }
    World b=a;b.settings.enableDataOrientedPredictor=true;
    double worst=0;
    for(int t=0;t<16;t++){
        // Mutations after the scratch buffer has already been populated.
        if(t==7){a.body(50).velocity.x+=2;b.body(50).velocity.x+=2;}
        if(t==9){a.settings.dt=1./90;b.settings.dt=1./90;}
        a.step();b.step();
        require(a.statistics().certifiedFreeFlight==1&&b.statistics().certifiedFreeFlight==1,"sparse path certificate failed");
        worst=std::max(worst,compare(a,b));
    }
    require(worst<1e-10,"certified free flight differs from AoS predictor");
    std::printf("PASS certified rotating free flight, 512 bodies 16 steps, max delta %.4g\n",worst);
    // Case B: insertion after first step + static obstacle and contact solver.
    a=World{};a.settings.enableSpatialBroadphase=true;
    a.settings.iterations=11;a.settings.postIterations=8;
    a.addBox({0,-.5,0},{18,1,18},0);
    for(int j=0;j<4;j++)for(int x=0;x<4;x++)for(int z=0;z<4;z++){
        int id=a.addBox({(x-1.5)*1.03,.51+j*1.02,(z-1.5)*1.03},{1,1,1},1);
        a.body(id).restitution=.04;
    }
    b=a;b.settings.enableDataOrientedPredictor=true;
    worst=0;int maxContacts=0;
    for(int t=0;t<110;t++){
        if(t==20){
            int x=a.addSphere({0,7,0},.6,50);int y=b.addSphere({0,7,0},.6,50);
            a.body(x).velocity={1,-3,.2};b.body(y).velocity={1,-3,.2};
        }
        a.step();b.step();worst=std::max(worst,compare(a,b));
        maxContacts=std::max(maxContacts,b.statistics().contacts);
    }
    require(maxContacts>0,"contact-rich test recorded no contacts");
    require(worst<1e-8,"contact solver trajectory differs with packed predictor");
    std::printf("PASS contact-rich rigid-body stack, max contacts %d, max delta %.4g\n",maxContacts,worst);
    // Case C: articulated joint solving with dynamic bodies and static anchor.
    a=World{};a.settings.gravity={0,-9.81,0};
    int root=a.addBox({0,6,0},{.4,.4,.4},0);
    int tip=a.addSphere({1,3,0},.4,1);
    a.addDistanceJoint(root,tip,{0,6,0},{1,3,0},std::sqrt(10.));
    b=a;b.settings.enableDataOrientedPredictor=true;
    worst=0;
    for(int t=0;t<70;t++){a.step();b.step();worst=std::max(worst,compare(a,b));}
    require(worst<1e-9,"joint constraint trajectory differs");
    std::printf("PASS joint constraint and static anchor, max delta %.4g\n",worst);
    // Case D: full CCD and event-driven steps must preserve existing behavior.
    a=World{};a.settings.gravity={};a.settings.enableCCD=true;
    int wall=a.addBox({0,0,0},{.08,8,8},0);
    a.body(wall).angularVelocity={0,10,0};
    tip=a.addSphere({-5,0,0},.2,1);
    a.body(tip).velocity={1800,0,0};
    b=a;b.settings.enableDataOrientedPredictor=true;
    a.step();b.step();worst=compare(a,b);
    require(a.statistics().ccdEvents==b.statistics().ccdEvents&&a.statistics().ccdEvents>0,"CCD event divergence");
    require(worst<1e-9,"CCD trajectory differs");
    std::printf("PASS rotational CCD + data-oriented predictor, max delta %.4g\n",worst);
    // Case E: the flat occupancy grid must never certify an overlapping pair.
    a=World{};a.settings.enableCertifiedFreeFlight=true;
    a.settings.enableFlatFreeFlightCertificate=true;
    int one=a.addSphere({0,1,0},.7,1),two=a.addSphere({.2,1,0},.7,1);
    a.step();require(!a.statistics().certifiedFreeFlight,"flat table falsely certified an overlap");
    require(a.statistics().contacts>0,"flat table skipped contact generation");
    std::puts("PASS flat-grid overlap rejection");
    // Case F: move bodies via public mutable references, toggle modes,
    // and compare the flat grid against the original unordered_set reference.
    a=World{};a.settings.enableCertifiedFreeFlight=true;
    a.settings.gravity={0,-.4,0};
    for(int i=0;i<2500;i++){
        int id=a.addBox({double(i%50)*4,30+double(i/50)*4,5},{.5,.5,.5},1);
        a.body(id).velocity={.02*(i%3),-.05,.01*(i%5)};
    }
    b=a;b.settings.enableFlatFreeFlightCertificate=true;
    b.settings.enableDataOrientedPredictor=true;
    worst=0;
    for(int t=0;t<20;t++){
        if(t==5){a.body(11).velocity.x+=.1;b.body(11).velocity.x+=.1;}
        if(t==12){a.settings.dt=1./80;b.settings.dt=1./80;}
        a.step();b.step();
        require(a.statistics().certifiedFreeFlight==1&&b.statistics().certifiedFreeFlight==1,"flat-table false rejection");
        worst=std::max(worst,compare(a,b));
    }
    require(worst<1e-9,"flat occupancy grid changed free-flight trajectory");
    std::printf("PASS flat occupancy + reusable SoA buffers 2500 boxes 20 steps, delta %.4g\n",worst);
    // Case G: an oversized static floor cannot be erroneously certified.
    a=World{};a.settings.enableCertifiedFreeFlight=true;
    a.settings.enableFlatFreeFlightCertificate=true;
    a.addBox({0,-.5,0},{100,1,100},0);
    a.addSphere({0,.4,0},.6,1);
    a.step();require(!a.statistics().certifiedFreeFlight,"oversized floor falsely certified");
    require(a.statistics().contacts>0,"flat certificate prevented static contact");
    std::puts("PASS oversized static floor rejected");
    std::puts("Stage 11 data-layout parity: PASS");return 0;
}catch(const std::exception& e){std::fprintf(stderr,"Stage 11 FAIL: %s\n",e.what());return 1;}}
