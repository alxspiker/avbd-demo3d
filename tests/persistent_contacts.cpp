#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
using namespace avbd;
int main(){
 World original, cached;
 original.settings.enableSpatialBroadphase=cached.settings.enableSpatialBroadphase=true;
 original.settings.enableParallelNarrowphase=cached.settings.enableParallelNarrowphase=true;
 original.settings.enableParallelSolver=cached.settings.enableParallelSolver=true;
 original.settings.enableIslandSolver=cached.settings.enableIslandSolver=true;
 original.settings.parallelThreads=cached.settings.parallelThreads=4;
 cached.settings.enablePersistentManifoldStorage=true;
 auto populate=[](World& w){
  w.addBox({0,-.5,0},{50,1,50},0);
  for(int y=0;y<4;y++)for(int x=0;x<7;x++)for(int z=0;z<7;z++)
   w.addBox({(x-3)*1.015,.51+y*1.015,(z-3)*1.015},{1,1,1},1);
 };
 populate(original);populate(cached);
 double delta=0;int matched=0;
 for(int step=0;step<180;step++){
  original.step();cached.step();
  if(original.statistics().contacts!=cached.statistics().contacts ||
     original.statistics().manifolds!=cached.statistics().manifolds){
    std::printf("FAIL contact parity step %d\n",step);return 1;
  }
  matched+=cached.statistics().contacts;
  for(size_t i=0;i<original.bodies().size();i++){
   delta=std::max(delta,length(original.body((int)i).p-cached.body((int)i).p));
  }
 }
 std::printf("contact observations %d; maximum position difference %.12g\n",matched,delta);
 if(matched==0 || delta>1e-8){std::puts("FAIL persistent manifold parity");return 1;}
 std::puts("PASS persistent manifold parity");return 0;
}
