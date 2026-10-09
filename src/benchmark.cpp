#include "avbd3d.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
using namespace avbd;
int main(int argc,char**argv){
    const int side=argc>1?std::atoi(argv[1]):6;
    for(bool sleep:{false,true}){
        World w;w.settings.enableSleeping=sleep;w.settings.iterations=20;w.settings.postIterations=14;
        w.addBox({0,-.5,0},{50,1,50},0);
        for(int y=0;y<4;y++)for(int x=0;x<side;x++)for(int z=0;z<side;z++)
            w.addBox({(x-(side-1)/2.)*1.04,.52+y*1.05,(z-(side-1)/2.)*1.04},{1,1,1},1,.7);
        for(int i=0;i<240;i++)w.step();
        auto t0=std::chrono::steady_clock::now();
        for(int i=0;i<120;i++)w.step();
        auto t1=std::chrono::steady_clock::now();
        const double ms=1000*std::chrono::duration<double>(t1-t0).count()/120;
        std::printf("%s %d bodies: %.3f ms/step, %d sleeping at end, %d contacts\n",sleep?"sleep enabled":"sleep disabled",side*side*4,ms,w.statistics().sleepingBodies,w.statistics().contacts);
    }
}
