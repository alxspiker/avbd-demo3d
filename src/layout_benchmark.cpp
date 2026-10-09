// Contact-rich end-to-end World::step comparison: includes broadphase,
// narrowphase, full AVBD constraint solver, and the optional SoA predictor.
#include "avbd3d.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <stdexcept>
using namespace avbd;
int main(int argc,char**argv){
    const std::string mode=argc>1?argv[1]:"aos";
    const int frames=argc>2?std::atoi(argv[2]):60;
    if((mode!="aos"&&mode!="soa")||frames<1||frames>1000)
        throw std::invalid_argument("usage: avbd3d_layout_benchmark aos|soa [1..1000 frames]");
    World w;
    w.settings.enableDataOrientedPredictor=(mode=="soa");
    w.settings.enableSpatialBroadphase=true;
    w.settings.enableParallelNarrowphase=true;
    w.settings.enableParallelSolver=true;
    w.settings.enableIslandSolver=true;
    w.settings.parallelThreads=4;
    w.settings.iterations=13;
    w.settings.postIterations=9;
    w.addBox({0,-.5,0},{50,1,50},0);
    for(int zone=0;zone<4;zone++){
        const double cx=(zone%2?8.:-8.),cz=(zone/2?8.:-8.);
        for(int y=0;y<5;y++)for(int x=0;x<4;x++)for(int z=0;z<4;z++)
            w.addBox({cx+(x-1.5)*.94,.50+y*.94,cz+(z-1.5)*.94},
                {.91,.91,.91},1,.6,Quat::rotation({0,1,0},.01*((x+y+z)%3)));
        const int projectile=w.addSphere({cx-4.1,8.8,cz},.9,25,.4);
        w.body(projectile).velocity={5.1,-4.5,0};
    }
    double totalMs=0;
    int maxContacts=0,impactEvents=0;
    for(int t=0;t<frames;t++){
        const auto begin=std::chrono::steady_clock::now();
        w.step();
        totalMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        maxContacts=std::max(maxContacts,w.statistics().contacts);
        impactEvents+=w.statistics().impactEvents;
    }
    std::printf("mode,bodies,steps,mean_ms_step,max_contacts,impact_events,last_y\n");
    std::printf("%s,%zu,%d,%.6f,%d,%d,%.12f\n",mode.c_str(),w.bodies().size()-1,frames,totalMs/frames,maxContacts,impactEvents,w.body(1).p.y);
}
