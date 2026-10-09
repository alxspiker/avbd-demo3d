#include "avbd3d.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
using namespace avbd;
static void trial(int nx,int ny,bool parallel,int threads,int steps){
 World w;w.settings.enableParallelSolver=parallel;w.settings.parallelThreads=threads;
 w.settings.iterations=12;w.settings.postIterations=8;
 w.addBox({0,-.5,0},{100,1,100},0);
 for(int y=0;y<ny;y++)for(int x=0;x<nx;x++)for(int z=0;z<nx;z++){
  w.addBox({(x-(nx-1)*.5)*1.01,.51+y*1.02,(z-(nx-1)*.5)*1.01},{1,1,1},1,.7);
 }
 for(int i=0;i<60;i++)w.step();
 auto start=std::chrono::steady_clock::now();
 int pairs=0,colors=0;
 for(int i=0;i<steps;i++){w.step();pairs+=w.statistics().pairs;colors=std::max(colors,w.statistics().solverColors);}
 double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/steps;
 double energy=0;
 for(auto& b:w.bodies())if(b.dynamic())energy+=length2(b.velocity);
 std::printf("%s threads %d bodies %d ms/step %.4f broadphase-pairs/step %.1f colors %d energy %.3f\n",parallel?"colored":"serial",threads,nx*nx*ny,ms,double(pairs)/steps,colors,energy);
}
int main(int argc,char**argv){int nx=argc>1?std::atoi(argv[1]):8,ny=argc>2?std::atoi(argv[2]):4,steps=argc>3?std::atoi(argv[3]):20;
 trial(nx,ny,false,1,steps);trial(nx,ny,true,1,steps);trial(nx,ny,true,2,steps);trial(nx,ny,true,4,steps);
}
