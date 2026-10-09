#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace avbd;
static void openTrace(const World& w,const char* title,int stage,double dt){
 std::printf("{\"title\":\"%s\",\"stage\":%d,\"dt\":%.12g,\"boxes\":[",title,stage,dt);
 for(size_t i=0;i<w.bodies().size();i++){
  const auto &b=w.bodies()[i];
  std::printf("%s{\"size\":[%.7g,%.7g,%.7g],\"shape\":%d}",i?",":"",b.half.x*2,b.half.y*2,b.half.z*2,b.shape==Shape::Sphere?1:0);
 }
 std::printf("],\"frames\":[");
}
static void printFrame(const World& w,int i){
 std::printf("%s{\"b\":[",i?",":"");
 for(const auto& b:w.bodies())std::printf("%s[%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g]",b.id?",":"",b.p.x,b.p.y,b.p.z,b.q.w,b.q.x,b.q.y,b.q.z);
 std::printf("],\"contacts\":%d,\"colors\":%d,\"ccd\":%d,\"unresolved\":%d}",w.statistics().contacts,w.statistics().solverColors,w.statistics().ccdEvents,w.statistics().ccdUnresolved);
}
static World makeShoot(bool ccd){
 World w;w.settings.dt=1./30;w.settings.gravity={};w.settings.enableCCD=ccd;
 w.settings.iterations=25;w.settings.postIterations=20;
 w.addBox({0,2.5,0},{.08,5,9},0,0); // thin, opaque wall
 for(int i=0;i<16;i++){int id=w.addSphere({0,-800.0-i,0},.26,1,0);w.body(id).restitution=.7;}
 return w;
}
static void launch(World& w,int round){
 for(int k=0;k<2;k++){
  const int projectile=1+round*2+k;
  const double sign=(round+k)%2?1.:-1.;
  Body& s=w.body(projectile);
  s.p={sign*2.5,1.35+((round+k)%3)*1.0,(k?2.0:-2.0)};
  s.velocity={-sign*50,0,0};
  s.angularVelocity={};s.sleeping=false;s.quietTime=0;
 }
}
static void captureCCD(){
 World off=makeShoot(false),on=makeShoot(true);
 std::printf("{\"title\":\"06 / SWEPT TIME OF IMPACT VS DISCRETE\",\"stage\":6,\"dt\":%.12g,\"boxes\":[",off.settings.dt);
 for(size_t i=0;i<off.bodies().size();i++){
  const auto& b=off.bodies()[i];std::printf("%s{\"size\":[%.5g,%.5g,%.5g],\"shape\":%d}",i?",":"",b.half.x*2,b.half.y*2,b.half.z*2,b.shape==Shape::Sphere?1:0);
 }
 std::printf("],\"frames\":[");
 int n=0,totalOn=0,totalOff=0,unresolved=0;
 for(int tick=0;tick<120;tick++){
  if(tick%12==0 && tick/12<8){launch(off,tick/12);launch(on,tick/12);n+=2;}
  if(tick){off.step();on.step();totalOn+=on.statistics().ccdEvents;unresolved+=on.statistics().ccdUnresolved;}
  for(int i=1;i<=n;i++){
   const int r=(i-1)/2,k=(i-1)%2;
   const double sign=(r+k)%2?1.:-1.;
   const auto &b=off.body(i);
   if(b.p.y>-100&&b.p.y<10 && sign*b.p.x < -.31)++totalOff;
  }
  std::printf("%s{\"off\":[",tick?",":"");
  for(const auto& b:off.bodies())std::printf("%s[%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g]",b.id?",":"",b.p.x,b.p.y,b.p.z,b.q.w,b.q.x,b.q.y,b.q.z);
  std::printf("],\"on\":[");
  for(const auto& b:on.bodies())std::printf("%s[%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g]",b.id?",":"",b.p.x,b.p.y,b.p.z,b.q.w,b.q.x,b.q.y,b.q.z);
  std::printf("],\"shots\":%d,\"ccd_events\":%d,\"unresolved\":%d}",n,totalOn,unresolved);
 }
 std::printf("],\"summary\":{\"ccd_events\":%d,\"unresolved\":%d,\"discrete_crossing_samples\":%d}}",totalOn,unresolved,totalOff);
 std::fprintf(stderr,"CCD capture: shots=%d, CCD events=%d, unresolved=%d, discrete crossing samples=%d\n",n,totalOn,unresolved,totalOff);
}
static void captureParallel(){
 World w;w.settings.enableParallelSolver=true;w.settings.parallelThreads=4;
 w.settings.iterations=18;w.settings.postIterations=12;
 w.addBox({0,-.5,0},{45,1,45},0);
 for(int y=0;y<5;y++)for(int x=0;x<9;x++)for(int z=0;z<9;z++){
  const double px=(x-4)*.99+.035*std::sin(x*2+z*7+y),pz=(z-4)*.99+.035*std::cos(x*8+z*3+y);
  const int id=w.addBox({px,3.0+y*1.12,pz},{.9,.9,.9},1,.6,Quat::rotation({.1,1,.1},.018*(x+z+y)));
  w.body(id).restitution=.11;
 }
 const int p=w.addSphere({0,28,0},1.45,14,.3);w.body(p).restitution=.18;
 openTrace(w,"07 / GRAPH-COLORED PARALLEL SOLVER",7,w.settings.dt*4);
 int maxColors=0,maxContacts=0, impacts=0;
 for(int i=0;i<360;i++){
  if(i)for(int j=0;j<4;j++){w.step();maxColors=std::max(maxColors,w.statistics().solverColors);maxContacts=std::max(maxContacts,w.statistics().contacts);impacts+=w.statistics().impactEvents;}
  printFrame(w,i);
 }
 std::printf("],\"summary\":{\"max_colors\":%d,\"max_contacts\":%d,\"impacts\":%d}}",maxColors,maxContacts,impacts);
 std::fprintf(stderr,"parallel capture: dynamic=%zu, max colors=%d, max contacts=%d, impacts=%d\n",w.bodies().size()-1,maxColors,maxContacts,impacts);
}
int main(int argc,char**argv){if(argc!=2)return std::fprintf(stderr,"usage: avbd3d_milestones ccd|parallel\n"),1;
 const std::string opt=argv[1];if(opt=="ccd")captureCCD();else if(opt=="parallel")captureParallel();else return 2;}
