#include <chrono>
#include <cstdio>
#include "inspectloop_safety/velocity_gate.hpp"
using namespace inspectloop_safety;
int main(int argc, char**) {
  ScanData s; s.angle_min = 0; s.angle_increment = 1e-9; s.angle_max = 719e-9;
  s.range_min = 0.12; s.range_max = 12; s.ranges.assign(720, 5.0f);
  CoverageParams p; auto t0 = std::chrono::steady_clock::now();
  SectorCoverage c = sectorCoverage(s, 0.0, p.forwardHalfAngle(), 0.95, 0.035, 0.0175);  // forward only
  double dt = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
  std::printf("forward sector: %.2f s observed=%d\n", dt, c.observed);
  if (argc > 1) { t0 = std::chrono::steady_clock::now(); auto a = assessScan(s, p);
    std::printf("assessScan: %.2f s validity=%d\n", std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count(), (int)a.validity);}
}
