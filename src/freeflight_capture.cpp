#include "avbd3d.h"
#include <cmath>
#include <cstdio>
using namespace avbd;
int main(){
    constexpr int n=10000,frames=120,substeps=6;
    World world;world.settings.enableCertifiedFreeFlight=true;
    world.settings.gravity={0,-2,0};
    world.settings.iterations=7;world.settings.postIterations=4;
    for(int i=0;i<n;i++){
        const int x=i%50,y=(i/50)%20,z=i/1000;
        int id=world.addBox({(x-25)*3.0,45.+y*3.,(z-5)*3.0},{.7,.7,.7},1);
        world.body(id).velocity={.02*(i%5),-.8+.03*(i%3),.015*(i%7)};
    }
    std::puts("{\"stage\":10,\"bodies\":10000,\"rendered_sample\":1250,\"frames\":[");
    for(int frame=0;frame<frames;frame++){
        int certified=1;
        for(int s=0;s<substeps;s++){
            world.step();
            certified &= world.statistics().certifiedFreeFlight;
        }
        if(!certified){std::fprintf(stderr,"Certificate failed at frame %d\n",frame);return 1;}
        if(frame)std::puts(",");
        std::printf("{\"time\":%.3f,\"contacts\":%d,\"certified\":%d,\"points\":[",(frame+1)*substeps/120.,world.statistics().contacts,certified);
        for(int s=0;s<1250;s++){
            int id=s*8;const Body& b=world.body(id);
            if(s)std::putchar(',');
            std::printf("[%0.4f,%0.4f,%0.4f,%d]",b.p.x,b.p.y,b.p.z,(id/1000));
        }
        std::printf("]}");
    }
    std::puts("]}");
}
