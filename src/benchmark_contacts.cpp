#include "avbd3d.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
using namespace avbd;

// Stable dense, sustained-contact workload shared by legacy/current comparisons.
// This benchmark measures complete World::step, not map-manipulation alone.
int main(int argc,char** argv){
    const int trials=argc>1?std::atoi(argv[1]):5;
    const int steps=argc>2?std::atoi(argv[2]):100;
    if(trials<1||trials>100||steps<1||steps>10000){
        std::fputs("usage: avbd3d_contact_benchmark [1..100 trials] [1..10000 timed steps]\n",stderr);
        return 1;
    }
    std::puts("trial,steps,ms_per_step,average_contacts,average_manifolds");
    for(int trial=0;trial<trials;trial++){
        World w;
        w.settings.enableSpatialBroadphase=true;
        w.settings.enableParallelNarrowphase=true;
        w.settings.enableParallelSolver=true;
        w.settings.enableIslandSolver=true;
        w.settings.parallelThreads=4;
        w.settings.iterations=7;
        w.settings.postIterations=4;
        w.addBox({0,-.5,0},{100,1,100},0);
        for(int y=0;y<3;y++)for(int x=0;x<9;x++)for(int z=0;z<9;z++)
            w.addBox({(x-4)*1.015,.51+y*1.015,(z-4)*1.015},{1,1,1},1);
        for(int i=0;i<30;i++)w.step();
        long long contactSum=0,manifoldSum=0;
        auto start=std::chrono::steady_clock::now();
        for(int i=0;i<steps;i++){
            w.step();
            contactSum+=w.statistics().contacts;
            manifoldSum+=w.statistics().manifolds;
        }
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/steps;
        std::printf("%d,%d,%.6f,%.3f,%.3f\n",trial,steps,ms,double(contactSum)/steps,double(manifoldSum)/steps);
        if(contactSum==0)throw std::runtime_error("contact benchmark lost all contacts");
    }
}
