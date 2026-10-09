#include "avbd3d.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
using namespace avbd;
static int checks=0;
static void require(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
static double delta(Vec3 a,Vec3 b){return length(a-b);}
static World makeScattered(int n,bool overlappingX=false){
    World w;w.settings.gravity={0,-.3,0};w.settings.iterations=7;w.settings.postIterations=4;
    w.addBox({0,-2,0},{240,1,240},0);
    // The nearly coincident X extents force 1-axis SAP to scan large active lists,
    // whereas a 3D tree can reject pairs on their Y/Z extents.
    for(int i=0;i<n;i++){
        const int z=i%125,y=i/125;
        const double x=overlappingX?0.0:(i%21)*2.0-20.0;
        const auto id=w.addBox({x,2.0+y*3.1,z*2.1-130.}, {0.9,0.9,0.9},1.0);
        w.body(id).velocity={0.01*(i%3),0,0};
    }
    return w;
}
static World makeContacts(){
    World w;w.settings.iterations=12;w.settings.postIterations=8;
    w.addBox({0,-.5,0},{40,1,40},0);
    for(int c=0;c<3;c++)for(int y=0;y<3;y++)for(int x=0;x<3;x++)for(int z=0;z<3;z++){
        double px=(c-1)*11.+(x-1)*1.01;
        const int id=w.addBox({px,.5+y*1.0,(z-1)*1.01},{1,1,1},1,.7,
                             Quat::rotation({0,1,0},.02*(x+y+z)));
        w.body(id).velocity={.03*(x-1),0,.02*(z-1)};
    }
    return w;
}
static void compare(World& a,World& b,int steps,double tolerance){
    for(int i=0;i<steps;i++){
        a.step();b.step();
        require(a.statistics().pairs==b.statistics().pairs,"candidate generation changed narrowphase call count");
        require(a.statistics().contacts==b.statistics().contacts,"candidate generation changed contact count");
        require(a.statistics().collisionIslands==b.statistics().collisionIslands,"candidate generation changed islands");
        for(size_t j=0;j<a.bodies().size();j++){
            const Body& x=a.body(static_cast<int>(j));const Body& y=b.body(static_cast<int>(j));
            require(delta(x.p,y.p)<tolerance,"body position parity diverged");
            require(delta(x.velocity,y.velocity)<tolerance,"body velocity parity diverged");
            require(std::abs(x.q.w-y.q.w)<tolerance,"body quaternion parity diverged");
        }
    }
}
int main(){try{
    {
        World legacy=makeContacts(),bvh=legacy;
        bvh.settings.enableSpatialBroadphase=true;
        compare(legacy,bvh,25,1e-9);
        std::puts("BVH and one-axis SAP collision/contact and trajectory parity: PASS");
    }
    {
        World serial=makeContacts(),parallel=serial;
        serial.settings.enableSpatialBroadphase=true;
        parallel.settings.enableSpatialBroadphase=true;
        parallel.settings.enableParallelNarrowphase=true;
        parallel.settings.parallelThreads=4;
        compare(serial,parallel,25,1e-9);
        std::puts("Parallel narrowphase deterministic merge and trajectories: PASS");
    }
    {
        World baseline=makeContacts(),islands=baseline;
        baseline.settings.enableSpatialBroadphase=true;
        islands.settings.enableSpatialBroadphase=true;
        islands.settings.enableParallelSolver=true;
        islands.settings.enableIslandSolver=true;
        islands.settings.parallelThreads=4;
        compare(baseline,islands,20,1e-8);
        require(islands.statistics().collisionIslands>=3,"separate piles merged through static floor");
        require(islands.statistics().largestIsland>=20,"multi-body island missing");
        std::puts("Independent island parallelism and static-floor separation: PASS");
    }
    {
        // Joint-only connection must merge dynamic bodies, even without collision.
        World w;w.settings.gravity={};w.settings.enableIslandSolver=true;
        int a=w.addSphere({0,0,0},.2,1),b=w.addSphere({3,0,0},.2,1);
        w.addSphere({10,0,0},.2,1);
        w.addDistanceJoint(a,b,{0,0,0},{3,0,0},3);
        w.step();
        require(w.statistics().collisionIslands==2 && w.statistics().largestIsland==2,"joint connected-components incorrect");
        std::puts("Enabled joints join connected islands: PASS");
    }
    {
        World legacy=makeScattered(2000,true),bvh=legacy;
        bvh.settings.enableSpatialBroadphase=true;
        compare(legacy,bvh,2,1e-10);
        std::puts("2000 sparse bodies, identical X projections, exact parity: PASS");
    }
    {
        World legacy,bvh;
        legacy.settings.gravity={};legacy.settings.enableCCD=true;
        int wall=legacy.addBox({0,0,0},{.08,6,6},0);
        int sphere=legacy.addSphere({-3,1,0},.15,1);
        legacy.body(sphere).velocity={900,0,0};
        bvh=legacy;bvh.settings.enableSpatialBroadphase=true;
        compare(legacy,bvh,1,1e-8);
        require(bvh.statistics().ccdEvents>0,"BVH CCD missed known thin-wall event");
        std::puts("Swept BVH CCD maintains thin-wall behavior: PASS");
    }
    {
        World a=makeContacts();a.settings.enableParallelSolver=true;a.settings.enableIslandSolver=true;
        a.settings.enableParallelNarrowphase=true;a.settings.enableSpatialBroadphase=true;
        a.settings.parallelThreads=4;
        for(int step=0;step<30;step++)a.step();
        for(auto& b:a.bodies())require(std::isfinite(b.p.x)&&std::isfinite(b.p.y)&&std::isfinite(b.p.z),"non-finite stage 9 state");
        std::puts("All Stage 9 acceleration options together: PASS");
    }
    std::printf("Stage 9: %d checks PASSED\n",checks);
    return 0;
}catch(const std::exception& ex){std::fprintf(stderr,"stage 9 FAIL: %s (check %d)\n",ex.what(),checks);return 1;}}
