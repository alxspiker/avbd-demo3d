#include "avbd3d.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>
using namespace avbd;
static constexpr double PI=3.141592653589793;
int main(int argc,char**argv){
    if(argc!=2){std::fprintf(stderr,"Usage: avbd3d_stages 1|2|3|4|5\n");return 1;}
    const int stage=std::atoi(argv[1]);
    if(stage<1||stage>5){std::fprintf(stderr,"Stage 1..5 required\n");return 1;}
    World w;w.settings.iterations=20;w.settings.postIterations=14;
    std::string title;
    if(stage==1){
        title="01 / ELASTIC IMPACTS";
        w.addBox({0,-.5,0},{26,1,26},0);
        for(int y=0;y<2;y++)for(int x=0;x<7;x++)for(int z=0;z<6;z++){
            double px=(x-3)*1.25+.17*std::sin(x*3+z*11+y*7), pz=(z-2.5)*1.25+.17*std::cos(x*7+z*3+y);
            int id=w.addBox({px,5.5+y*2.15+.3*std::sin(x*3+z*5),pz},{.88,.88,.88},1,.55,Quat::rotation({.2,1,.35},.12*(x+z+y)));
            w.body(id).restitution=.50;
            w.body(id).angularVelocity={.06*(z-2),.07*(x-3),.05*(y?1:-1)};
        }
    }else if(stage==2){
        title="02 / FAST SPHERES VS THIN WALL";
        w.settings.gravity={0,0,0};w.settings.enableAdaptiveSubsteps=true;w.settings.maxMotionFraction=.25;
        w.addBox({0,3.8,0},{.09,8,14},0);
        for(int i=0;i<22;i++){
            bool right=i%2;double sign=right?1:-1;
            int id=w.addSphere({sign*(8+18*(i/2)),2+(i%5)*.85,(i%7-3)*1.65},.28,.9,0);
            w.body(id).velocity={-sign*30,0,0};w.body(id).restitution=.65;
        }
    }else if(stage==3){
        title="03 / SPHERE DEMOLITION";
        w.addBox({0,-.5,0},{160,1,160},0);
        for(int layer=0;layer<2;layer++)for(int y=0;y<11;y++)for(int z=0;z<6;z++){
            const double x=(layer-.5)*.77,py=.29+y*.59,pz=(z-2.5)*.81;
            int id=w.addBox({x,py,pz},{.72,.56,.77},1,.63);
            w.body(id).restitution=.12;
        }
        int s=w.addSphere({-11,3.0,0},1.32,32,.5);
        w.body(s).velocity={17,0,0};w.body(s).restitution=.08;
        s=w.addSphere({13,4.8,1.8},.85,30,.4);
        w.body(s).velocity={-10,0,0};w.body(s).restitution=.12;
    }else if(stage==4){
        title="04 / ATTACHED WRECKING BALL";
        w.addBox({0,-.5,0},{56,1,32},0);
        int prev=w.addSphere({-8.0,13.5,0},.28,0);
        // Taut diagonal chain: positions selected to form a freely swinging pendulum.
        for(int i=1;i<=13;i++){
            const Vec3 p{-8.0+i*.65,13.5-i*.51,0};
            int id=w.addSphere(p,i==13?.85:.23,(i==13?2.0:1.5),.45);
            w.body(id).restitution=.18;
            double br=std::numeric_limits<double>::infinity(); // Keep the wrecking ball attached during demolition; fracture is tested separately.
            const Vec3 prior=w.body(prev).p;
            w.addDistanceJoint(prev,id,prior,p,length(prior-p),std::numeric_limits<double>::infinity(),br);
            prev=id;
        }
        for(int y=0;y<7;y++)for(int z=0;z<5;z++){
            int id=w.addBox({-12.8,.35+y*.7,(z-2)*.87},{1.1,.65,.84},1,.7);
            w.body(id).restitution=.05;
        }
    }else if(stage==5){
        title="05 / SLEEP AND WAKE";
        w.settings.enableSleeping=true;
        w.addBox({0,-.5,0},{50,1,50},0);
        for(int y=0;y<4;y++)for(int x=0;x<6;x++)for(int z=0;z<6;z++){
            const double px=(x-2.5)*1.04+.02*std::sin(x*11+z*7+y),pz=(z-2.5)*1.04+.02*std::cos(x*4+z*13+y);
            w.addBox({px,.52+y*1.05,pz},{1,1,1},1,.7);
        }
        int s=w.addSphere({0,195,0},1.12,24,.4);
        w.body(s).restitution=.08;
    }
    std::printf("{\"title\":\"%s\",\"dt\":%.12g,\"stage\":%d,\"boxes\":[",title.c_str(),w.settings.dt*4,stage);
    for(size_t i=0;i<w.bodies().size();i++){
        const Body& b=w.body(static_cast<int>(i));
        std::printf("%s{\"size\":[%.5g,%.5g,%.5g],\"shape\":%d}",i?",":"",b.half.x*2,b.half.y*2,b.half.z*2,b.shape==Shape::Sphere?1:0);
    }
    std::printf("],\"links\":[");
    for(size_t i=0;i<w.joints().size();i++){const auto& j=w.joints()[i];std::printf("%s[%d,%d]",i?",":"",j.a,j.b);}
    std::printf("],\"frames\":[");
    int totalImpacts=0,totalBreaks=0,maxSubsteps=1;
    for(int frame=0;frame<360;frame++){
        if(frame)for(int sub=0;sub<4;sub++){
            w.step();totalImpacts+=w.statistics().impactEvents;totalBreaks+=w.statistics().brokenJoints;
            maxSubsteps=std::max(maxSubsteps,w.statistics().ccdSubsteps);
        }
        std::printf("%s{\"b\":[",frame?",":"");
        for(const Body& b:w.bodies())
            std::printf("%s[%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g]",b.id?",":"",b.p.x,b.p.y,b.p.z,b.q.w,b.q.x,b.q.y,b.q.z);
        std::printf("],\"j\":[");
        for(size_t i=0;i<w.joints().size();i++)std::printf("%s%d",i?",":"",w.joints()[i].enabled?1:0);
        std::printf("],\"impacts\":%d,\"broken\":%d,\"substeps\":%d,\"sleeping\":%d}",totalImpacts,totalBreaks,maxSubsteps,w.statistics().sleepingBodies);
    }
    std::printf("]}");
}
