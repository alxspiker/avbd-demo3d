#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
using namespace avbd;
int main(){
 int fails=0;
 auto check=[&](bool pass,const char*name,double value){std::printf("%s %s (%.7g)\n",pass?"PASS":"FAIL",name,value);fails+=!pass;};
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;
  w.addBox({0,0,0},{.06,8,8},0);
  int s=w.addSphere({-5,0,0},.18,1,0);w.body(s).velocity={1800,0,0};w.body(s).restitution=.9;
  w.step();
  check(w.body(s).p.x<-.18,"sphere vs thin box 1800 m/s no tunnelling",w.body(s).p.x);
  check(w.statistics().ccdEvents>0,"sphere-box registered CCD impact",w.statistics().ccdEvents);
  check(w.statistics().ccdUnresolved==0,"sphere-box event budget not exhausted",w.statistics().ccdUnresolved);
 }
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;
  int a=w.addSphere({-5,0,0},.2,1,0),b=w.addSphere({5,0,0},.2,1,0);
  w.body(a).velocity={1500,0,0};w.body(b).velocity={-1500,0,0};
  w.body(a).restitution=w.body(b).restitution=.9;
  w.step();
  check(w.body(a).p.x<w.body(b).p.x,"two 1500 m/s spheres preserve order",w.body(a).p.x);
  check(w.statistics().ccdEvents>0,"sphere-sphere continuous collision event",w.statistics().ccdEvents);
 }
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;
  w.addBox({0,0,0},{.08,7,7},0);
  int id=w.addBox({-5,0,0},{.35,.35,.35},1,0);
  w.body(id).velocity={1600,0,0};w.step();
  check(w.body(id).p.x<0,"1600 m/s box blocked by thin wall",w.body(id).p.x);
  check(w.statistics().ccdEvents>0,"box-box swept SAT reports impact",w.statistics().ccdEvents);
 }
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;
  w.addBox({0,0,0},{.08,7,7},0,0,Quat::rotation({0,1,0},.35));
  int s=w.addSphere({-5,0,0},.18,1,0);
  w.body(s).velocity={1800,0,0};w.step();
  const Vec3 n=w.body(0).q.rotate({1,0,0});
  const double signedDist=dot(n,w.body(s).p);
  check(signedDist<-(.04+.18-.01),"sphere stays outside rotated wall surface",signedDist);
  check(w.statistics().ccdEvents>0,"rotated sphere-box event",w.statistics().ccdEvents);
 }
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;
  w.addBox({0,0,0},{.08,7,7},0);int s=w.addSphere({-5,9,0},.18,1,0);
  w.body(s).velocity={1800,0,0};w.step();
  check(w.body(s).p.x>0,"grazing non-contact not falsely stopped",w.body(s).p.x);
  check(w.statistics().ccdEvents==0,"miss has zero CCD events",w.statistics().ccdEvents);
 }
 {
  World serial,parallel;parallel.settings.enableParallelSolver=true;parallel.settings.parallelThreads=3;
  serial.addBox({0,-.5,0},{80,1,80},0);parallel.addBox({0,-.5,0},{80,1,80},0);
  for(int x=0;x<7;x++)for(int z=0;z<7;z++){
   const Vec3 p{(x-3)*2.3,3+(x%3)*.05,(z-3)*2.3};
   serial.addBox(p,{.8,.8,.8},1);parallel.addBox(p,{.8,.8,.8},1);
  }
  double maxDelta=0;
  for(int tick=0;tick<240;tick++){
   serial.step();parallel.step();
   for(int i=1;i<50;i++)maxDelta=std::max(maxDelta,length(serial.body(i).p-parallel.body(i).p));
  }
  check(maxDelta<1e-6,"colored solver agrees with serial for independent contacts",maxDelta);
  check(parallel.statistics().solverColors>0,"colored solver generated scheduling groups",parallel.statistics().solverColors);
 }
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;w.settings.maxCCDSteps=1;
  w.addBox({0,0,0},{.06,8,8},0);
  int s=w.addSphere({-5,0,0},.2,1,0);
  w.body(s).velocity={1400,0,0};
  w.step();
  check(w.statistics().ccdUnresolved>0,"CCD event budget exhaustion is reported",w.statistics().ccdUnresolved);
 }
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;
  int a=w.addBox({0,0,0},{1,1,1},1);
  w.body(a).angularVelocity={0,8,0};
  int s=w.addSphere({-5,0,0},.2,1);
  w.body(s).velocity={1400,0,0};
  w.step();
  check(w.statistics().ccdUnsupportedRotation>0,"rotating box TOI limitation is explicit",w.statistics().ccdUnsupportedRotation);
 }
 {
  World w;w.settings.gravity={};w.settings.enableCCD=true;
  w.addBox({0,0,0},{.08,8,8},0);
  int s=w.addSphere({-5,0,0},.2,1,0);
  w.body(s).velocity={100000,0,0};w.body(s).restitution=.85;
  w.step();
  check(w.body(s).p.x<0,"100000 units/s linear sphere not tunneled",w.body(s).p.x);
  check(!w.statistics().ccdUnresolved,"100000 units/s resolves within budget",w.statistics().ccdUnresolved);
 }
 {
  World w;w.settings.enableParallelSolver=true;w.settings.parallelThreads=4;
  w.settings.iterations=20;w.settings.postIterations=12;
  w.addBox({0,-.5,0},{30,1,30},0);
  for(int y=0;y<4;y++)for(int x=0;x<6;x++)for(int z=0;z<6;z++)
   w.addBox({(x-2.5)*1.01,.52+y*1.02,(z-2.5)*1.01},{1,1,1},1);
  double maxPen=0;int maxColors=0;
  for(int tick=0;tick<240;tick++){
   w.step();
   maxPen=std::max(maxPen,w.statistics().maxPenetration);
   maxColors=std::max(maxColors,w.statistics().solverColors);
  }
  check(maxColors>=2,"dense pile requires multiple dependency colors",maxColors);
  check(maxPen<.25,"parallel dense pile bounded maximum penetration",maxPen);
 }
 return fails?1:0;
}
