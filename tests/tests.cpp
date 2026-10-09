#include "avbd3d.h"
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <chrono>
using namespace avbd;
static int failures=0;
static void check(bool condition,const char* name,const std::string& details){
    std::printf("%-28s %s %s\n",name,condition?"PASS":"FAIL",details.c_str());
    if(!condition)failures++;
}
static World ground(){World w;w.addBox({0,-.5,0},{100,1,100},0);return w;}
int main(){
    try{
        {World w;int id=w.addBox({0,10,0},{1,1,1},1);for(int i=0;i<120;i++)w.step();double y=w.body(id).p.y;
            check(std::abs(y-(10-9.81*.5))<.07,"gravity freefall", "y="+std::to_string(y));}
        {World w;w.settings.gravity={0,0,0};int id=w.addBox({0,0,0},{1,2,3},1);w.body(id).angularVelocity={0,2,0};for(int i=0;i<120;i++)w.step();
            double n=w.body(id).q.w*w.body(id).q.w+w.body(id).q.x*w.body(id).q.x+w.body(id).q.y*w.body(id).q.y+w.body(id).q.z*w.body(id).q.z;
            double angle=length(w.body(id).q.log());check(std::abs(n-1)<1e-8&&std::abs(angle-2)<.03,"free rigid rotation","angle="+std::to_string(angle)+" qnorm="+std::to_string(n));}
        {World w=ground();int id=w.addBox({0,2,0},{1,1,1},1);double maxPen=0;for(int i=0;i<720;i++){w.step();maxPen=std::max(maxPen,w.statistics().maxPenetration);}
            const auto& b=w.body(id);check(std::abs(b.p.y-.5)<.06 && std::abs(b.velocity.y)<.12 && maxPen<.12,"resting box","y="+std::to_string(b.p.y)+" vy="+std::to_string(b.velocity.y)+" maxPen="+std::to_string(maxPen));}
        {World w=ground();int ids[3];for(int i=0;i<3;i++)ids[i]=w.addBox({0,.51+i*1.02,0},{1,1,1},1);
            double maxPen=0;for(int i=0;i<720;i++){w.step();maxPen=std::max(maxPen,w.statistics().maxPenetration);}
            std::string desc;bool ok=true;for(int i=0;i<3;i++){const auto& b=w.body(ids[i]);double goal=.5+i;ok &= std::abs(b.p.y-goal)<.12 && std::abs(b.velocity.y)<.2;desc+=" y"+std::to_string(i)+"="+std::to_string(b.p.y);}
            check(ok&&maxPen<.2,"three-box stack",desc+" maxPen="+std::to_string(maxPen));}
        {World w=ground();int id=w.addBox({0,1.2,0},{1,1,1},1,.6,Quat::rotation({0,0,1},.35));for(int i=0;i<480;i++)w.step();
            const auto& b=w.body(id);check(std::isfinite(b.p.y)&&std::isfinite(b.q.w)&&b.p.y>-.4&&b.p.y<2,"tilted box impact","y="+std::to_string(b.p.y)+" omega="+std::to_string(length(b.angularVelocity)));}
        {World w;w.settings.gravity={0,0,0};int a=w.addBox({-1.2,0,0},{1,1,1},1);int b=w.addBox({1.2,0,0},{1,1,1},1);
            w.body(a).velocity={2,0,0};w.body(b).velocity={-2,0,0};double maxPen=0;
            for(int i=0;i<180;i++){w.step();maxPen=std::max(maxPen,w.statistics().maxPenetration);}
            double totalPx=w.body(a).velocity.x+w.body(b).velocity.x;
            check(w.body(a).p.x<w.body(b).p.x&&std::abs(totalPx)<.1&&maxPen<.12,"head-on collision momentum",
                  "xA="+std::to_string(w.body(a).p.x)+" xB="+std::to_string(w.body(b).p.x)+" vxSum="+std::to_string(totalPx)+" maxPen="+std::to_string(maxPen));}
        {World w=ground();int id=w.addBox({0,.51,0},{1,1,1},1,1.0);w.body(id).velocity={3,0,0};
            for(int i=0;i<360;i++)w.step();double v=w.body(id).velocity.x;
            check(std::abs(v)<.35,"ground friction braking","vx="+std::to_string(v)+" x="+std::to_string(w.body(id).p.x));}
        {World w=ground();int ids[8];for(int i=0;i<8;i++)ids[i]=w.addBox({0,.51+i*1.02,0},{1,1,1},1);
            double maxPen=0;for(int i=0;i<960;i++){w.step();maxPen=std::max(maxPen,w.statistics().maxPenetration);}
            bool ok=true;for(int i=0;i<8;i++)ok &= std::abs(w.body(ids[i]).p.y-(i+.5))<.5;
            check(ok&&maxPen<.25,"eight-box long stack","topY="+std::to_string(w.body(ids[7]).p.y)+" maxPen="+std::to_string(maxPen));}
        {World w=ground();int id=w.addBox({0,2,0},{1,1,1},1,.6,Quat::rotation({0,0,1},.48));
            double maxW=0;for(int i=0;i<480;i++){w.step();maxW=std::max(maxW,length(w.body(id).angularVelocity));}
            check(maxW>.1,"off-center contact torque","peak angular velocity="+std::to_string(maxW));}
        {World w;w.settings.gravity={0,0,0};int a=w.addBox({0,0,0},{1,1,1},1,.5,Quat::rotation({0,1,0},.45));
            int b=w.addBox({.85,.15,.05},{1,1,1},1,.5,Quat::rotation({0,0,1},.35));
            double maxPen=0;for(int i=0;i<120;i++){w.step();maxPen=std::max(maxPen,w.statistics().maxPenetration);}
            check(std::isfinite(w.body(a).p.x)&&std::isfinite(w.body(b).p.x)&&maxPen<.3,
                  "rotated OBB contact","maxPen="+std::to_string(maxPen));}
        {World w=ground();int id=w.addBox({0,.51,0},{1,1,1},1,0.0);w.body(id).velocity={3,0,0};
            for(int i=0;i<240;i++)w.step();double vx=w.body(id).velocity.x;
            check(std::abs(vx-3)<.05,"frictionless sliding","vx="+std::to_string(vx));}
        {World w=ground();int id=w.addBox({0,.0,0},{1,1,1},1);for(int i=0;i<180;i++)w.step();
            check(std::abs(w.body(id).p.y-.5)<.05,"deep initial overlap recovery","y="+std::to_string(w.body(id).p.y));}
        {World w=ground();w.settings.dt=1.0/240.;int id=w.addBox({0,1.5,0},{1,1,1},1);
            for(int i=0;i<1440;i++)w.step();check(std::abs(w.body(id).p.y-.5)<.05,
                  "240 Hz fixed-step stability","y="+std::to_string(w.body(id).p.y));}
        {World w=ground();for(int x=0;x<8;x++)for(int z=0;z<8;z++)w.addBox({x*1.5,1+((x+z)%3)*1.1,z*1.5},{1,1,1},1);
            auto t0=std::chrono::steady_clock::now();for(int i=0;i<120;i++)w.step();auto t1=std::chrono::steady_clock::now();
            double sec=std::chrono::duration<double>(t1-t0).count();check(std::isfinite(w.statistics().maxSpeed)&&sec<30,"64-body stability/performance","120 steps sec="+std::to_string(sec)+" contacts="+std::to_string(w.statistics().contacts));}
    }catch(const std::exception& e){std::printf("EXCEPTION: %s\n",e.what());return 2;}
    std::printf("RESULT: %d failures\n",failures);return failures?1:0;
}
