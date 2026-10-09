#include "avbd3d.h"
#include <array>
#include <cstdio>
#include <cmath>
using namespace avbd;

// Stage 8 visual regression: two IDENTICAL physical scenes except for CCD.
// Each event runs six actual 120 Hz simulation ticks. The recording repeats
// each sampled pose for six display frames (6x slow replay) to expose impact.
struct Shot{double omega,x,y;};
static const std::array<Shot,8> shots{{
    {80,1.4,.7},{80,1.8,.7},{80,1.8,1.0},{120,1.4,.7},
    {120,1.8,1.0},{120,1.4,1.3},{-80,1.8,-.7},{-120,1.4,-1.3}
}};
static void pose(const World& w){
    std::printf("[");
    for(const Body& b:w.bodies()){
        std::printf("%s[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]",b.id?",":"",b.p.x,b.p.y,b.p.z,b.q.w,b.q.x,b.q.y,b.q.z);
    }
    std::printf("]");
}
static void setup(World& w,const Shot& s,bool ccd){
    w.settings.gravity={};
    w.settings.enableCCD=ccd;
    w.settings.maxCCDSteps=80;
    w.settings.iterations=24;
    w.settings.postIterations=20;
    const int rod=w.addBox({0,0,0},{4,.12,.12},100,0.3);
    w.body(rod).angularVelocity={0,0,s.omega};
    const int sphere=w.addSphere({s.x,s.y,0},.22,1,0.3);
    w.body(sphere).restitution=.7;
}
int main(){
    std::printf("{\"stage\":8,\"title\":\"08 / ROTATIONAL CCD: VERIFIED IMPACT\",\"display_fps\":24,\"step_dt\":%.12g,\"repeat_frames\":6,\"boxes\":[{\"size\":[4,0.12,0.12],\"shape\":0},{\"size\":[0.44,0.44,0.44],\"shape\":1}],\"frames\":[",1./120.);
    bool first=true;
    int totalCcd=0,totalImpacts=0,misses=0;
    for(size_t shot=0;shot<shots.size();shot++){
        World off,on;
        setup(off,shots[shot],false);setup(on,shots[shot],true);
        int ccd=0,ccdImpacts=0,plainImpacts=0,unresolved=0;
        for(int tick=0;tick<6;tick++){
            if(tick){
                off.step();on.step();
                ccd+=on.statistics().ccdEvents;
                ccdImpacts+=on.statistics().impactEvents;
                plainImpacts+=off.statistics().impactEvents;
                unresolved+=on.statistics().ccdUnresolved;
            }
            for(int hold=0;hold<6;hold++){
                if(!first)std::printf(",");first=false;
                std::printf("{\"shot\":%zu,\"tick\":%d,\"omega\":%.6g,\"target\":[%.6g,%.6g],\"off\":",shot,tick,shots[shot].omega,shots[shot].x,shots[shot].y);
                pose(off);
                std::printf(",\"on\":");pose(on);
                std::printf(",\"ccd\":%d,\"ccd_impacts\":%d,\"off_impacts\":%d,\"unresolved\":%d,\"speed_off\":%.6g,\"speed_on\":%.6g}",ccd,ccdImpacts,plainImpacts,unresolved,length(off.body(1).velocity),length(on.body(1).velocity));
            }
        }
        totalCcd+=ccd;totalImpacts+=ccdImpacts;
        if(plainImpacts==0 && ccdImpacts>0)++misses;
        std::fprintf(stderr,"shot %zu omega %.0f sphere(%.1f,%.1f) off_impact=%d ccd_impact=%d TOI=%d unresolved=%d final_displacement_off=%.3f on=%.3f\n",shot,shots[shot].omega,shots[shot].x,shots[shot].y,plainImpacts,ccdImpacts,ccd,unresolved,length(off.body(1).p-Vec3{shots[shot].x,shots[shot].y,0}),length(on.body(1).p-Vec3{shots[shot].x,shots[shot].y,0}));
    }
    std::printf("],\"summary\":{\"ccd_events\":%d,\"ccd_impacts\":%d,\"scenes_where_discrete_misses\":%d}}",totalCcd,totalImpacts,misses);
    std::fprintf(stderr,"total_rotational_toi=%d impact_impulses=%d scenarios_discrete_misses=%d\n",totalCcd,totalImpacts,misses);
}
