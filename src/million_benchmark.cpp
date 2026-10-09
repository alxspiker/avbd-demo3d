#include "avbd3d.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
using namespace avbd;
int main(int argc,char** argv){
    const int n=argc>1?std::atoi(argv[1]):10000;
    const int steps=argc>2?std::atoi(argv[2]):3;
    const std::string mode=argc>3?argv[3]:"certified";
    if(n<1||n>1000000||steps<1||steps>100||
       (mode!="certified"&&mode!="bvh"&&mode!="soa"&&mode!="soa_bvh"&&mode!="flat"&&mode!="soa_flat")){
        std::fprintf(stderr,"usage: avbd3d_million_benchmark <1..1000000 bodies> <1..100 steps> [certified|bvh|soa|soa_bvh|flat|soa_flat]\n");return 2;
    }
    auto start=std::chrono::steady_clock::now();
    World w;w.settings.enableCertifiedFreeFlight=(mode=="certified" || mode=="soa" || mode=="flat" || mode=="soa_flat");
    w.settings.enableDataOrientedPredictor=(mode=="soa" || mode=="soa_bvh" || mode=="soa_flat");
    w.settings.enableFlatFreeFlightCertificate=(mode=="flat" || mode=="soa_flat");
    w.settings.enableSpatialBroadphase=true;
    w.settings.iterations=7;w.settings.postIterations=4;
    w.settings.gravity={0,-.31,0};
    // One million actual 3D box rigid-body structs in the engine, no static ground.
    // Disjoint boxes with tiny varied velocities. This is intentionally free flight.
    for(int i=0;i<n;i++){
        const int x=i%100,y=(i/100)%100,z=i/10000;
        const int id=w.addBox({x*4.,y*4.+100.,z*4.},{.5,.5,.5},1.);
        w.body(id).velocity={.003*(i%3),-.02,.002*(i%5)};
    }
    auto built=std::chrono::steady_clock::now();
    double maxstep=0,totalstep=0;
    for(int s=0;s<steps;s++){
        auto begin=std::chrono::steady_clock::now();
        w.step();
        double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        maxstep=std::max(maxstep,elapsed);totalstep+=elapsed;
        if((mode=="certified"||mode=="soa"||mode=="flat"||mode=="soa_flat")&&!w.statistics().certifiedFreeFlight)
            throw std::runtime_error("sparse certificate rejected; do not count as successful fast-path benchmark");
    }
    const Body& first=w.body(0);const Body& last=w.body(n-1);
    std::printf("bodies,steps,mode,build_ms,mean_ms_step,max_ms_step,contacts,last_certified,first_y,last_y\n");
    std::printf("%d,%d,%s,%.3f,%.3f,%.3f,%d,%d,%.9f,%.9f\n",n,steps,mode.c_str(),
        std::chrono::duration<double,std::milli>(built-start).count(),totalstep/steps,maxstep,
        w.statistics().contacts,w.statistics().certifiedFreeFlight,first.p.y,last.p.y);
}
