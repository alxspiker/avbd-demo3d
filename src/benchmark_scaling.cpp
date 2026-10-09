#include "avbd3d.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
using namespace avbd;
static World scenario(int count,const char* type){
    World w;
    w.settings.iterations=7;w.settings.postIterations=4;
    w.settings.gravity={0,-.5,0};
    if(std::strcmp(type,"sparse")==0){
        // All X intervals overlap, but Y/Z do not: an intentionally difficult
        // workload for a one-axis sweep. This measures sparse spatial indexing.
        for(int i=0;i<count;i++){
            int x=i%125,y=i/125;
            int id=w.addBox({0,5+y*2.,(x-62)*2.},{.8,.8,.8},1);
            w.body(id).velocity={.03*(i%5),-.01,0};
        }
    }else if(std::strcmp(type,"islands")==0){
        // Each 4-body pile is independent even when all share the same plane.
        w.addBox({0,-.6,0},{500,1,500},0);
        for(int i=0;i<count;i++){
            int island=i/4,layer=i%4;
            int gx=island%50,gz=island/50;
            int id=w.addBox({(gx-25)*5.,.5+layer*.99,(gz-25)*5.},{1,1,1},1,.7);
            w.body(id).velocity={0,0,0};
        }
    }else throw std::invalid_argument("mode must be sparse or islands");
    return w;
}
static void trial(int count,const char* scene,int steps,int variant){
    World w=scenario(count,scene);
    if(variant>=1)w.settings.enableSpatialBroadphase=true;
    if(variant>=2){w.settings.enableParallelNarrowphase=true;w.settings.enableParallelSolver=true;w.settings.enableIslandSolver=true;w.settings.parallelThreads=4;}
    const auto begin=std::chrono::steady_clock::now();
    long long pairs=0,broad=0,contacts=0,islands=0;
    int largest=0;
    for(int i=0;i<steps;i++){
        w.step();
        pairs+=w.statistics().pairs;
        broad+=w.statistics().broadphaseCandidates;
        contacts+=w.statistics().contacts;
        islands+=w.statistics().collisionIslands;
        largest=std::max(largest,w.statistics().largestIsland);
    }
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()/steps;
    std::printf("%s,%d,%s,%d,%.4f,%.0f,%.0f,%.0f,%.0f,%d\n",scene,count,
        variant==0?"SAP/serial":variant==1?"BVH/serial":"BVH/4threads",steps,ms,
        double(broad)/steps,double(pairs)/steps,double(contacts)/steps,double(islands)/steps,largest);
}
int main(int argc,char** argv){
    int n=argc>1?std::atoi(argv[1]):1000;
    const char* scene=argc>2?argv[2]:"sparse";
    int steps=argc>3?std::atoi(argv[3]):3;
    if(n<=0||steps<=0||n>30000)return std::fprintf(stderr,"usage: avbd3d_scaling_benchmark <1..30000 bodies> [sparse|islands] [steps>0]\n"),1;
    std::puts("scene,bodies,configuration,steps,ms_per_step,broadphase_candidates,narrowphase_pairs,contacts,islands,largest_island");
    try{for(int variant=0;variant<3;variant++)trial(n,scene,steps,variant);}
    catch(const std::exception& e){return std::fprintf(stderr,"benchmark error: %s\n",e.what()),1;}
}
