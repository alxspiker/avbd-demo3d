#include "avbd3d.h"
#include <cstdio>
using namespace avbd;
int main(){
    World w;w.settings.gravity={0,0,0};
    w.addBox({0,0,0},{.1,3,3},0);
    int id=w.addBox({-2,0,0},{.3,.3,.3},1);
    w.body(id).velocity={400,0,0};
    w.step();
    std::printf("High-speed CCD probe (UNSUPPORTED): bullet x after one 1/120 s step = %.4f, contacts=%d; it crossed a thin wall.\n",w.body(id).p.x,w.statistics().contacts);
}
