// Stage 11 Kaggle capture: every projected point comes from an actual World::step Body.
// The sparse zero-contact benchmark is explicitly NOT dense million-body AVBD.
#include "avbd3d.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace avbd;
namespace fs = std::filesystem;

int main(int argc, char** argv) {
    try {
        if (argc != 5) {
            std::cerr << "Usage: avbd3d_million_video_capture <bodies 1..1000000> <frames 2..120> <steps_per_frame 1..10> <output_dir>\n";
            return 2;
        }
        const int n = std::stoi(argv[1]), frames = std::stoi(argv[2]), perFrame = std::stoi(argv[3]);
        if (n < 1 || n > 1000000 || frames < 2 || frames > 120 || perFrame < 1 || perFrame > 10)
            throw std::runtime_error("invalid body/frame count");
        const fs::path output = argv[4];
        fs::create_directories(output);
        constexpr int W = 1280, H = 720, CHANNELS = 3;
        std::vector<std::uint16_t> raster(static_cast<std::size_t>(W)*H*CHANNELS, 0);
        World world;
        world.settings.enableCertifiedFreeFlight = true;
        world.settings.enableFlatFreeFlightCertificate = true;
        world.settings.enableSpatialBroadphase = true;
        world.settings.iterations = 7;
        world.settings.postIterations = 4;
        world.settings.gravity = {0, -9.81, 0};
        const int zLayers = (n + 9999) / 10000;
        for (int i=0; i<n; ++i) {
            const int x = i%100, y = (i/100)%100, z = i/10000;
            const Vec3 center{(x-49.5)*4., 130.+y*4.13, (z-(zLayers-1)*.5)*4.};
            const int id = world.addBox(center, {.5,.5,.5}, 1.0);
            world.body(id).velocity = {.003*(i%3), -55., .002*(i%5)};
        }
        if (static_cast<int>(world.bodies().size()) != n) throw std::runtime_error("body creation mismatch");
        std::ofstream stats(output/"physics_stats.jsonl");
        if (!stats) throw std::runtime_error("could not create physics statistics");
        stats << std::setprecision(12);
        const auto begin = std::chrono::steady_clock::now();
        for (int frame=0; frame<frames; ++frame) {
            // Frame zero is true t=0; subsequent frames execute the actual C++ physics engine.
            if (frame > 0) for (int sub=0; sub<perFrame; ++sub) {
                world.step();
                if (world.statistics().certifiedFreeFlight != 1 || world.statistics().contacts != 0)
                    throw std::runtime_error("zero-contact certificate failed: this video cannot be represented as certified sparse physics");
            }
            std::fill(raster.begin(), raster.end(), 0);
            std::uint64_t visible=0;
            for (const Body& b : world.bodies()) {
                if (!std::isfinite(b.p.x) || !std::isfinite(b.p.y) || !std::isfinite(b.p.z))
                    throw std::runtime_error("non-finite rigid-body position");
                // A fixed oblique camera. All 1,000,000 bodies are rasterized, not merely a sample.
                const int px = static_cast<int>(std::lround(W*.5 + (b.p.x-b.p.z*.65)*1.58));
                const int py = static_cast<int>(std::lround(H*.48 - (b.p.y-320.)*.92 + (b.p.x+b.p.z)*.18));
                if (px<0 || px>=W || py<0 || py>=H) continue;
                const int zIndex = b.id / 10000;
                const int channel = std::min(2, (zIndex*3)/std::max(1,zLayers));
                std::uint16_t& count = raster[(static_cast<std::size_t>(py)*W + px)*CHANNELS + channel];
                if (count == UINT16_MAX) throw std::runtime_error("raster pixel density overflow");
                ++count;
                ++visible;
            }
            if (visible != static_cast<std::uint64_t>(n))
                throw std::runtime_error("camera cropped bodies; refusing to label frame as all bodies");
            char name[32];
            std::snprintf(name,sizeof(name),"density_%04d.bin",frame);
            std::ofstream raw(output/name, std::ios::binary);
            if (!raw.write(reinterpret_cast<const char*>(raster.data()),
                           static_cast<std::streamsize>(raster.size()*sizeof(std::uint16_t))))
                throw std::runtime_error("failed writing raster");
            const auto& st = world.statistics();
            const double t = frame * perFrame * world.settings.dt;
            stats << "{\"frame\":" << frame << ",\"bodies\":" << world.bodies().size()
                  << ",\"projected\":" << visible << ",\"sim_seconds\":" << t
                  << ",\"contacts\":" << st.contacts
                  << ",\"certified\":" << (frame==0?1:st.certifiedFreeFlight)
                  << ",\"first_y\":" << world.body(0).p.y
                  << ",\"middle_y\":" << world.body(n/2).p.y
                  << ",\"last_y\":" << world.body(n-1).p.y << "}\n";
            if (frame%8==0 || frame==frames-1)
                std::cerr << "Captured real physics frame " << frame+1 << "/" << frames
                          << ", body count=" << n << ", simulated time=" << t << " seconds\n";
        }
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        std::cerr << "PASS: " << n << " bodies, " << frames << " exact full-population frames, "
                  << (frames-1)*perFrame << " C++ World::step calls, " << elapsed
                  << " wall-clock seconds, zero contacts\n";
    } catch (const std::exception& e) {
        std::cerr << "Capture FAILED: " << e.what() << '\n';
        return 1;
    }
}
