#include "avbd3d.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <stdexcept>
#include <vector>
using namespace avbd;

static std::atomic<long long> heapAllocations{0};
static std::atomic<bool> counting{false};
void* operator new(std::size_t n){
    if(counting.load(std::memory_order_relaxed))heapAllocations.fetch_add(1,std::memory_order_relaxed);
    if(void* p=std::malloc(n?n:1))return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n){
    if(counting.load(std::memory_order_relaxed))heapAllocations.fetch_add(1,std::memory_order_relaxed);
    if(void* p=std::malloc(n?n:1))return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept {std::free(p);}
void operator delete(void* p,std::size_t) noexcept {std::free(p);}
void operator delete[](void* p) noexcept {std::free(p);}
void operator delete[](void* p,std::size_t) noexcept {std::free(p);}

static void densePile(World& w){
    w.settings.enableSpatialBroadphase=true;
    w.settings.enableParallelSolver=false;
    w.settings.enableIslandSolver=false;
    w.settings.enableParallelNarrowphase=false;
    w.settings.parallelThreads=1;
    w.settings.iterations=4;
    w.settings.postIterations=2;
    w.addBox({0,-.5,0},{100,1,100},0);
    for(int y=0;y<3;y++)for(int x=0;x<9;x++)for(int z=0;z<9;z++)
        w.addBox({(x-4)*1.015,.51+y*1.015,(z-4)*1.015},{1,1,1},1);
}

int main(){
    World w;densePile(w);
    for(int i=0;i<30;i++)w.step();
    long long total=0;
    for(int i=0;i<50;i++){
        const auto before=heapAllocations.load();
        counting.store(true,std::memory_order_release);
        w.step();
        counting.store(false,std::memory_order_release);
        total+=heapAllocations.load()-before;
        if(w.statistics().contacts!=972 || w.statistics().manifolds!=243){
            std::printf("FAIL pile lost contacts at step %d: %d %d\n",i,w.statistics().contacts,w.statistics().manifolds);
            return 1;
        }
    }
    std::printf("heap_allocations_for_50_dense_steps=%lld allocs_per_step=%.2f contacts=%d manifolds=%d\n",
        total,double(total)/50,w.statistics().contacts,w.statistics().manifolds);
    // This fixture isolates stable pairs: even with other engine allocations,
    // 50 steps should remain under 1500 allocations/step. Stage 12 uses 5624.
    if(total>75000){std::puts("FAIL excessive allocations in persistent dense contacts");return 1;}
}
