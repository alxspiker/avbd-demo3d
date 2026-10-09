#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
using namespace avbd;
int main(){
    World w;
    w.settings.enableSpatialBroadphase=true;
    w.settings.enableParallelNarrowphase=true;
    w.settings.enableParallelSolver=true;
    w.settings.enableIslandSolver=true;
    w.settings.parallelThreads=4;
    w.settings.iterations=13;w.settings.postIterations=9;
    w.settings.dt=1./120.;
    w.addBox({0,-.5,0},{50,1,50},0,.7);
    for(int zone=0;zone<4;zone++){
        const double cx=(zone%2?8.:-8.),cz=(zone/2?8.:-8.);
        for(int y=0;y<5;y++)for(int x=0;x<4;x++)for(int z=0;z<4;z++){
            const int id=w.addBox({cx+(x-1.5)*.94,.50+y*.94,cz+(z-1.5)*.94},
                {.91,.91,.91},1,.6,Quat::rotation({0,1,0},.01*((x+y+z)%3)));
            w.body(id).restitution=.08;
        }
        const int projectile=w.addSphere({cx-4.1,8.8,cz},.9,25,.4);
        w.body(projectile).velocity={5.1,-4.5,0};
        w.body(projectile).restitution=.15;
    }
    std::printf("{\"stage\":9,\"display_fps\":24,\"dt\":%.12g,\"shapes\":[",w.settings.dt);
    for(const Body& b:w.bodies()){
        std::printf("%s{\"size\":[%.6g,%.6g,%.6g],\"shape\":%d}",b.id?",":"",b.half.x*2,b.half.y*2,b.half.z*2,b.shape==Shape::Sphere?1:0);
    }
    std::printf("],\"frames\":[");
    int maxContacts=0,maxIslands=0,maxPairs=0,impacts=0;
    for(int frame=0;frame<240;frame++){
        if(frame)for(int tick=0;tick<2;tick++){
            w.step();
            maxContacts=std::max(maxContacts,w.statistics().contacts);
            maxIslands=std::max(maxIslands,w.statistics().collisionIslands);
            maxPairs=std::max(maxPairs,w.statistics().pairs);
            impacts+=w.statistics().impactEvents;
        }
        const auto& s=w.statistics();
        std::printf("%s{\"b\":[",frame?",":"");
        for(const Body& b:w.bodies()){
            std::printf("%s[%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g]",b.id?",":"",
                b.p.x,b.p.y,b.p.z,b.q.w,b.q.x,b.q.y,b.q.z);
        }
        std::printf("],\"contacts\":%d,\"pairs\":%d,\"broadphase\":%d,\"islands\":%d,\"largest\":%d,\"impacts\":%d}",
            s.contacts,s.pairs,s.broadphaseCandidates,s.collisionIslands,s.largestIsland,s.impactEvents);
    }
    std::printf("],\"summary\":{\"max_contacts\":%d,\"max_islands\":%d,\"max_pairs\":%d,\"impact_impulses\":%d,\"moving_bodies\":%zu}}",
        maxContacts,maxIslands,maxPairs,impacts,w.bodies().size()-1);
    std::fprintf(stderr,"Stage 9: %zu dynamic bodies, max contacts %d, max islands %d, max broad/narrow candidates %d, impulses %d\n",
        w.bodies().size()-1,maxContacts,maxIslands,maxPairs,impacts);
}
