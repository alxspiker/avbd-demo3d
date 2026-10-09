#include "avbd3d.h"
#include <array>
#include <cmath>
#include <cstdio>
using namespace avbd;
static int failures=0;
static void check(const char* name,bool pass,double value){
    std::printf("%s %s (%.9g)\n",pass?"PASS":"FAIL",name,value);
    if(!pass)++failures;
}
struct Shot{double omega,x,y;};
static World makeShot(Shot s,bool ccd){
    World w;
    w.settings.gravity={};
    w.settings.enableCCD=ccd;
    w.settings.maxCCDSteps=80;
    w.settings.iterations=24;
    w.settings.postIterations=20;
    int bar=w.addBox({0,0,0},{4,.12,.12},100,.3);
    w.body(bar).angularVelocity={0,0,s.omega};
    int ball=w.addSphere({s.x,s.y,0},.22,1,.3);
    w.body(ball).restitution=.7;
    return w;
}
int main(){
    const std::array<Shot,8> shots{{
       {80,1.4,.7},{80,1.8,.7},{80,1.8,1.0},{120,1.4,.7},
       {120,1.8,1.0},{120,1.4,1.3},{-80,1.8,-.7},{-120,1.4,-1.3}
    }};
    int cases=0;
    for(const Shot& s:shots){
        World off=makeShot(s,false),on=makeShot(s,true);
        off.step();on.step();
        const auto& a=off.body(1); const auto& b=on.body(1);
        const Vec3 initial{s.x,s.y,0};
        const double deltaOff=length(a.p-initial),deltaOn=length(b.p-initial);
        const bool pass=on.statistics().ccdEvents>0 && on.statistics().impactEvents>0 &&
            on.statistics().ccdUnresolved==0 && off.statistics().impactEvents==0 &&
            deltaOn>deltaOff+0.1 && length(b.velocity)>1 &&
            std::isfinite(b.p.x)&&std::isfinite(b.p.y)&&std::isfinite(b.q.w);
        if(!pass)std::printf("  details: omega %.3g target(%.2g,%.2g) offDelta %.6g onDelta %.6g events %d impulses %d unresolved %d\n",
            s.omega,s.x,s.y,deltaOff,deltaOn,on.statistics().ccdEvents,on.statistics().impactEvents,on.statistics().ccdUnresolved);
        ++cases;
        check("rotating rod produces a measured CCD-only impact",pass,deltaOn-deltaOff);
    }
    check("eight clockwise/counterclockwise CCD comparisons",cases==8,cases);
    {
        World w=makeShot({120,4,4},true);
        const Vec3 p=w.body(1).p;
        w.step();
        check("well-separated spinning rod does not report impact",w.statistics().ccdEvents==0 && w.statistics().impactEvents==0 && length(w.body(1).p-p)<1e-10,w.statistics().ccdEvents);
    }
    {
        World w;
        w.settings.gravity={};w.settings.enableCCD=true;
        int wall=w.addBox({0,0,0},{.08,6,6},0);
        w.body(wall).angularVelocity={0,12,0};
        int ball=w.addSphere({-5,0,0},.2,1,0);
        w.body(ball).velocity={1800,0,0};
        w.step();
        check("fast sphere versus rotating thin wall registers TOI",w.statistics().ccdEvents>0,w.statistics().ccdEvents);
        check("sphere stays on entry side of thin wall",w.body(ball).p.x<0,w.body(ball).p.x);
    }
    {
        World w;
        w.settings.gravity={};w.settings.enableCCD=true;
        int rod=w.addBox({0,0,0},{4,.12,.12},1);
        w.body(rod).angularVelocity={0,0,140};
        int ball=w.addSphere({1,1.1,0},.22,1,0);
        w.step();
        check("fast-spinning rod contacts an initially separated sphere",w.statistics().ccdEvents>0,w.statistics().ccdEvents);
        check("rotational contact leaves finite sphere state",std::isfinite(w.body(ball).p.y),w.body(ball).p.y);
    }
    std::printf("Stage 8 rotational CCD regression: %d failure(s)\n",failures);
    return failures?1:0;
}
