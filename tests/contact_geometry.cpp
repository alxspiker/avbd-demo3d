#include "avbd3d.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
using namespace avbd;
static uint32_t state=0x729CB135u;
static double rand01(){state=1664525u*state+1013904223u;return double(state)/4294967295.;}
int main(){
    World w;
    w.settings.enableSpatialBroadphase=true;
    w.settings.enableParallelNarrowphase=false;
    w.settings.parallelThreads=1;
    w.settings.iterations=7;
    w.settings.postIterations=4;
    w.addBox({0,-.5,0},{22,1,22},0);
    for(int i=0;i<72;i++){
        double x=(rand01()-.5)*8,z=(rand01()-.5)*8;
        double y=1+(i/24)*1.25+rand01()*.6;
        auto q=Quat::rotation(normalized({rand01()-.5,rand01()-.5,rand01()-.5}),rand01()*.7);
        int id=w.addBox({x,y,z},{.6+rand01()*.5,.65+rand01()*.4,.6+rand01()*.5},1,.62,q);
        w.body(id).angularVelocity={(rand01()-.5)*.9,(rand01()-.5)*.9,(rand01()-.5)*.9};
    }
    for(int i=0;i<8;i++){
        int id=w.addSphere({(rand01()-.5)*5,4+rand01()*2,(rand01()-.5)*5},.3+rand01()*.35,1);
        w.body(id).velocity={(rand01()-.5)*2,0,(rand01()-.5)*2};
    }
    long long totalContacts=0,totalManifolds=0;
    double sx=0,sy=0,sz=0,sq=0;
    int maxContacts=0;
    for(int frame=0;frame<120;frame++){
        w.step();
        totalContacts+=w.statistics().contacts;
        totalManifolds+=w.statistics().manifolds;
        maxContacts=std::max(maxContacts,w.statistics().contacts);
        for(size_t i=1;i<w.bodies().size();i++){
            const auto& b=w.body(static_cast<int>(i));
            if(!std::isfinite(b.p.x)||!std::isfinite(b.p.y)||!std::isfinite(b.p.z))return 1;
            sx+=b.p.x;sy+=b.p.y;sz+=b.p.z;sq+=b.q.w;
        }
    }
    std::printf("rotated_fixture contacts=%lld manifolds=%lld max_contacts=%d pos=(%.14g,%.14g,%.14g) quat=%.14g\n",
        totalContacts,totalManifolds,maxContacts,sx,sy,sz,sq);
    // Baseline captured independently with Stage 12's prior C++ narrowphase.
    const auto near=[](double actual,double expected){return std::abs(actual-expected)<1e-6;};
    if(totalContacts!=9067 || totalManifolds!=5178 || maxContacts!=159 ||
       !near(sx,1744.4879731936) || !near(sy,17522.445264383) ||
       !near(sz,3282.0902901584) || !near(sq,9192.7996087742)){
        std::puts("FAIL Stage 12 rotated contact geometry fixture changed");return 1;
    }
    return 0;
}
