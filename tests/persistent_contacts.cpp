#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
using namespace avbd;

static bool near(double actual,double expected,double tolerance=1e-5){
    return std::abs(actual-expected)<=tolerance;
}
static bool finite(const Vec3& v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
static void pile(World& w){
    w.settings.enableSpatialBroadphase=true;
    w.settings.enableParallelSolver=true;
    w.settings.enableIslandSolver=true;
    w.settings.parallelThreads=4;
    w.addBox({0,-.5,0},{50,1,50},0);
    for(int y=0;y<4;y++)for(int x=0;x<7;x++)for(int z=0;z<7;z++)
        w.addBox({(x-3)*1.015,.51+y*1.015,(z-3)*1.015},{1,1,1},1);
}
int main(){
    // Recorded before Stage 12 from the original full contact builder. This
    // protects actual solver output, not just parity between two new code paths.
    World serial,parallel;
    pile(serial);pile(parallel);
    serial.settings.enableParallelNarrowphase=false;
    parallel.settings.enableParallelNarrowphase=true;
    long long observations=0;
    double maxDelta=0;
    for(int step=0;step<180;step++){
        serial.step();parallel.step();
        if(serial.statistics().contacts!=parallel.statistics().contacts ||
           serial.statistics().manifolds!=parallel.statistics().manifolds){
            std::printf("FAIL narrowphase contact parity at step %d\n",step);return 1;
        }
        observations+=parallel.statistics().contacts;
        for(size_t i=0;i<serial.bodies().size();i++){
            maxDelta=std::max(maxDelta,length(serial.body(static_cast<int>(i)).p-parallel.body(static_cast<int>(i)).p));
        }
        if(step==30 || step==90 || step==179){
            if(parallel.statistics().contacts!=784 || parallel.statistics().manifolds!=196){
                std::printf("FAIL legacy contact fixture step %d: %d contacts, %d manifolds\n",step,
                    parallel.statistics().contacts,parallel.statistics().manifolds);
                return 1;
            }
        }
    }
    if(observations!=134456 || maxDelta>1e-8 ||
       !near(parallel.body(1).p.y,.4989979486059) ||
       !near(parallel.body(100).p.y,2.496994811763) ||
       !near(parallel.body(196).p.y,3.495994103237)){
        std::printf("FAIL original solver fixture: contacts=%lld parity=%.12g\n",observations,maxDelta);
        return 1;
    }
    // Contacts must expire and recreate when pairs separate and meet again.
    World changing;
    changing.settings.gravity={0,0,0};
    changing.settings.iterations=7;
    changing.settings.postIterations=4;
    changing.addSphere({-.2,0,0},.5,1);
    changing.addSphere({.2,0,0},.5,1);
    changing.body(0).velocity={-1,0,0};
    changing.body(1).velocity={1,0,0};
    bool first=false,expired=false,recreated=false;
    for(int i=0;i<320;i++){
        if(i==100){
            changing.body(0).velocity={2,0,0};
            changing.body(1).velocity={-2,0,0};
        }
        changing.step();
        int contacts=changing.statistics().contacts;
        if(i<40 && contacts>0)first=true;
        if(i>=40 && i<100 && contacts==0)expired=true;
        if(i>100 && contacts>0)recreated=true;
        if(!finite(changing.body(0).p)||!finite(changing.body(1).p)){
            std::puts("FAIL non-finite body in contact churn");return 1;
        }
    }
    if(!first || !expired || !recreated){
        std::printf("FAIL contact lifecycle: initial=%d expired=%d recreated=%d\n",first,expired,recreated);
        return 1;
    }
    std::printf("PASS single persistent contact path: legacy=%lld contacts, serial/parallel delta=%.12g, expire/recreate=OK\n",
        observations,maxDelta);
}
