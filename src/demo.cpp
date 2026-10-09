#include "avbd3d.h"
#include <cstdio>
#include <cstdlib>
#include <string>
using namespace avbd;
int main(int argc,char** argv){
    int frames=360;if(argc>1)frames=std::atoi(argv[1]);
    World w;w.addBox({0,-.5,0},{30,1,30},0);
    int ids[5];for(int i=0;i<5;i++)ids[i]=w.addBox({0,.55+1.1*i,0},{1,1,1},1);
    for(int i=0;i<frames;i++){
        w.step();if(i%60==59){const auto& stats=w.statistics();std::printf("t=%.2f contacts=%d pairs=%d penetration=%.6g",(i+1)*w.settings.dt,stats.contacts,stats.pairs,stats.maxPenetration);
            for(int id:ids)std::printf(" y%d=%.5f",id,w.body(id).p.y);std::printf("\n");}
    }return 0;
}
