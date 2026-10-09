#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
using namespace avbd;
int main(){int failures=0;auto check=[&](bool b,const char *s,double v){std::printf("%s %s: %.6g\n",b?"PASS":"FAIL",s,v);if(!b)failures++;};
 {World w;w.addBox({0,-.5,0},{30,1,30},0);int id=w.addBox({0,3,0},{1,1,1},1,0);w.body(id).restitution=.8;double peak=-100;bool hit=false;int events=0;for(int i=0;i<360;i++){w.step();events+=w.statistics().impactEvents;if(events>0)peak=std::max(peak,w.body(id).p.y);}check(events>0,"first bounce events",events);check(peak>1,"restitution rebound maximum",peak);}
 {World w;w.settings.gravity={0,0,0};int a=w.addBox({-2,0,0},{1,1,1},1,0),b=w.addBox({2,0,0},{1,1,1},1,0);w.body(a).velocity={5,0,0};w.body(b).velocity={-5,0,0};w.body(a).restitution=w.body(b).restitution=.85;int events=0;for(int i=0;i<90;i++){w.step();events+=w.statistics().impactEvents;}check(events>0,"head-on bounce events",events);check(w.body(a).velocity.x<-.5&&w.body(b).velocity.x>.5,"head-on reversal",w.body(a).velocity.x);}
 {World w;w.settings.gravity={0,0,0};w.settings.enableAdaptiveSubsteps=true;
    w.addBox({0,0,0},{.1,4,4},0);int id=w.addBox({-2,0,0},{.3,.3,.3},1,0);
    w.body(id).velocity={400,0,0};w.step();
    check(w.statistics().ccdSubsteps>=10,"adaptive subdivision engaged",w.statistics().ccdSubsteps);
    check(w.body(id).p.x<0,"fast projectile blocked by thin wall",w.body(id).p.x);
 }
 {World w;w.addBox({0,-.5,0},{20,1,20},0);int id=w.addSphere({0,3,0},.5,1,.4);
   for(int i=0;i<480;i++)w.step();check(std::abs(w.body(id).p.y-.5)<.07,"sphere resting on box floor",w.body(id).p.y);
 }
 {World w;w.settings.gravity={0,0,0};int a=w.addSphere({-2,0,0},.5,1,0),b=w.addSphere({2,0,0},.5,1,0);w.body(a).restitution=w.body(b).restitution=.8;w.body(a).velocity={3,0,0};w.body(b).velocity={-3,0,0};for(int i=0;i<200;i++)w.step();
   check(w.body(a).p.x <w.body(b).p.x,"sphere-sphere no pass-through",w.body(a).p.x);
   check(w.body(a).velocity.x<0 && w.body(b).velocity.x>0,"sphere-sphere bounce",w.body(a).velocity.x);
 }
 {World w;w.settings.gravity={0,0,0};int box=w.addBox({0,0,0},{1,1,1},0);int sphere=w.addSphere({-2,0,0},.35,1,0);w.body(sphere).velocity={4,0,0};for(int i=0;i<100;i++)w.step();
   check(w.body(sphere).p.x <0,"sphere-box block",w.body(sphere).p.x);
 }
 {World w;int anchor=w.addBox({0,5,0},{.3,.3,.3},0), bob=w.addSphere({2,3,0},.4,1);
    w.addDistanceJoint(anchor,bob,{0,5,0},{2,3,0},2.5);
    double maxLength=0;for(int i=0;i<420;i++){w.step();maxLength=std::max(maxLength,length(w.body(anchor).p-w.body(bob).p));}
    check(maxLength<2.9,"distance joint prevents runaway stretch",maxLength);
    check(std::isfinite(w.body(bob).p.x)&&std::isfinite(w.body(bob).p.y),"distance joint numerical stability",w.body(bob).p.y);
 }
 {World w;w.settings.gravity={0,-9.81,0};int anchor=w.addBox({0,5,0},{.3,.3,.3},0),bob=w.addSphere({0,3,0},.4,1);
    int joint=w.addDistanceJoint(anchor,bob,{0,5,0},{0,3,0},1,10000,1);
    int broke=0;for(int i=0;i<100;i++){w.step();broke+=w.statistics().brokenJoints;}
    check(broke>0&&!w.joints()[joint].enabled,"fracture threshold breaks joint",broke);
 }
 {World w;w.settings.enableSleeping=true;w.addBox({0,-.5,0},{12,1,12},0);
    int id=w.addBox({0,.51,0},{1,1,1},1);for(int i=0;i<600;i++)w.step();
    check(w.body(id).sleeping,"stable box enters sleep",w.body(id).quietTime);
    int projectile=w.addBox({-4,.5,0},{.5,.5,.5},2);w.body(projectile).velocity={6,0,0};
    bool woke=false;for(int i=0;i<300;i++){w.step();woke|=!w.body(id).sleeping;}
    check(woke,"impact wakes sleeping box",w.body(id).p.x);
 }

 // Regression: all 22 projectiles must stay on their entry side of a thin wall.
 // Check every physics tick, not merely the video frames, at the speed/positions
 // used in the recorded scene. Substeps themselves may still fail for other cases.
 {World w;w.settings.gravity={0,0,0};w.settings.enableAdaptiveSubsteps=true;
   w.settings.maxMotionFraction=.25;
   w.addBox({0,3.8,0},{.09,8,14},0);
   int ids[22];int signs[22];
   for(int i=0;i<22;i++){
      signs[i]=(i%2)?1:-1;
      ids[i]=w.addSphere({static_cast<double>(signs[i])*(8+18*(i/2)),2+(i%5)*.85,(i%7-3)*1.65},.28,.9,0);
      w.body(ids[i]).velocity={-static_cast<double>(signs[i])*30,0,0};
      w.body(ids[i]).restitution=.65;
   }
   int crossed=0;double minSigned=1e30;
   for(int tick=0;tick<1436;tick++){
      w.step();
      for(int i=0;i<22;i++){
         double signedX=signs[i]*w.body(ids[i]).p.x;
         minSigned=std::min(minSigned,signedX);
         if(signedX<0.045+.28-0.005)crossed++;
      }
   }
   check(crossed==0,"all 22 fast spheres remain outside wall (every tick)",crossed);
   check(minSigned>=0.32,"wall clearance remains nonpenetrating",minSigned);
 }
 // A lone tipped cube should rotate down to a face, unlike cubes that can be
 // physically supported at an angle by nearby bodies in a dense pile.
 {int bad=0;double minAlign=1;
   for(double angle:{0.15,0.35,0.55,0.75,0.95,1.15}){
     World w;w.settings.iterations=20;w.settings.postIterations=14;
     w.addBox({0,-.5,0},{40,1,40},0);
     int id=w.addBox({0,4,0},{1,1,1},1,.6,Quat::rotation({1,.2,.5},angle));
     for(int tick=0;tick<1000;tick++)w.step();
     const Body &b=w.body(id);
     double align=std::max({std::abs(b.q.rotate({1,0,0}).y),std::abs(b.q.rotate({0,1,0}).y),std::abs(b.q.rotate({0,0,1}).y)});
     minAlign=std::min(minAlign,align);
     if(align<.99||std::abs(b.p.y-.5)>.02)bad++;
   }
   check(bad==0,"isolated tilted cubes settle face-down",minAlign);
 }
 return failures?1:0;}
