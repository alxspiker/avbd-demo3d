#include "avbd3d.h"
#include <cstdio>
#include <vector>
#include <cmath>
using namespace avbd;
int main(){World w;std::vector<int> ids;ids.push_back(w.addBox({0,-.5,0},{30,1,30},0));
for(int y=0;y<4;y++)for(int x=0;x<6;x++)for(int z=0;z<4;z++){
 double px=(x-2.5)*1.65+.18*std::sin(y*7+x*3+z),pz=(z-1.5)*1.65+.18*std::cos(y*3+x+z*5);
 double py=6.5+y*2.25+.23*std::sin(x*7+z*11);
 int id=w.addBox({px,py,pz},{.95,.95,.95},1,.55,Quat::rotation({.3,1,.2},.11*(x+z+y)));
 w.body(id).angularVelocity={.2*(z-1),.3*(x-2),.1*(y-2)};ids.push_back(id);
}
printf("{\"dt\":%.12g,\"boxes\":[",w.settings.dt*2);
for(size_t i=0;i<ids.size();i++){auto &b=w.body(ids[i]);printf("%s{\"size\":[%.4g,%.4g,%.4g]}",i?",":"",b.half.x*2,b.half.y*2,b.half.z*2);}printf("],\"frames\":[");
for(int f=0;f<360;f++){if(f){w.step();w.step();}printf("%s[",f?",":"");for(size_t i=0;i<ids.size();i++){auto &b=w.body(ids[i]);printf("%s[%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g]",i?",":"",b.p.x,b.p.y,b.p.z,b.q.w,b.q.x,b.q.y,b.q.z);}printf("]");}printf("]}");}
