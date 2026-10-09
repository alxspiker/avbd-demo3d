// Minimal CPU structure-of-arrays free-flight throughput experiment.
// Not a complete rigid-body engine and NOT a replacement for World::step.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>
int main(int argc,char** argv){
    const int n=argc>1?std::atoi(argv[1]):1000000;
    const int steps=argc>2?std::atoi(argv[2]):100;
    if(n<1||n>2000000||steps<1||steps>10000)return 2;
    std::vector<double> px(n),py(n,100),pz(n),vx(n),vy(n,-.02),vz(n);
    for(int i=0;i<n;i++){px[i]=i%103;pz[i]=i%47;vx[i]=(i%5)*.1;vz[i]=(i%7)*.03;}
    constexpr double dt=1.0/120.0, g=-.31;
    const auto begin=std::chrono::steady_clock::now();
    for(int step=0;step<steps;step++){
        for(int i=0;i<n;i++){
            px[i]+=vx[i]*dt;
            py[i]+=vy[i]*dt+g*dt*dt;
            pz[i]+=vz[i]*dt;
            vy[i]+=g*dt;
        }
    }
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()/steps;
    const double expected=100.-.02*dt*steps+g*dt*dt*(double(steps)*(steps+1)/2.);
    if(!std::isfinite(px[n-1])||std::abs(py[n-1]-expected)>1e-8)throw std::runtime_error("SoA integration analytic failure");
    std::printf("SoA_translation_only, bodies=%d, steps=%d, ms_per_step=%.6f, sample_y=%.9f, result=PASS\n",n,steps,ms,py[n-1]);
}
