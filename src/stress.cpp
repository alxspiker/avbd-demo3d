#include "avbd3d.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace avbd;
int main(int argc,char** argv){
    int side=argc>1?std::atoi(argv[1]):10,steps=argc>2?std::atoi(argv[2]):120;
    if(side<1||side>30||steps<1||steps>10000)return 2;
    World w;w.addBox({0,-.5,0},{double(side*2+2),1,double(side*2+2)},0);
    int total=0;
    for(int y=0;y<side;y++)for(int x=0;x<side;x++)for(int z=0;z<side;z++){
        w.addBox({(x-(side-1)*.5)*1.05,1.05*y+.55,(z-(side-1)*.5)*1.05},{1,1,1},1);total++;
    }
    double maxPen=0;auto t0=std::chrono::steady_clock::now();
    for(int i=0;i<steps;i++){w.step();maxPen=std::max(maxPen,w.statistics().maxPenetration);}
    double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
    std::printf("dynamic_bodies=%d steps=%d elapsed_s=%.3f ms_per_step=%.3f maxPen=%.6g contacts=%d pairs=%d maxSpeed=%.3f\n",total,steps,elapsed,elapsed*1000./steps,maxPen,w.statistics().contacts,w.statistics().pairs,w.statistics().maxSpeed);
    return std::isfinite(w.statistics().maxSpeed)?0:1;
}
