// Read recorded frames and call the production filter, without a ROS node.
#include "ats_rog_map/planning/ground_projection_fusion.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) {throw std::runtime_error("usage: terrain_confirmation_replay frames.txt");}
    std::ifstream input(argv[1]);
    if (!input) {throw std::runtime_error("cannot open frames");}
    const std::vector<int> candidates{1, 2, 4, 8, 12, 16};
    std::vector<ats_rog_map::planning::TerrainSpikeFilter> filters;
    for (int n : candidates) {filters.emplace_back(n, 100);}
    std::cout << "frame,mono_s,stamp_ns,N,raw_occupied,retained_occupied,suppressed,soft_free,unknown,resets\n";
    std::cout << std::setprecision(17);
    nav_msgs::msg::OccupancyGrid frame;
    std::int64_t stamp;
    double mono, yaw;
    std::size_t count = 0;
    while (input >> mono) {
      if (!(input >> stamp >> frame.header.frame_id >> frame.info.width >> frame.info.height >>
        frame.info.resolution >> frame.info.origin.position.x >> frame.info.origin.position.y >> yaw))
      {throw std::runtime_error("incomplete frame header");}
      frame.header.stamp.sec = stamp / 1000000000;
      frame.header.stamp.nanosec = stamp % 1000000000;
      frame.info.origin.orientation.z = std::sin(yaw * .5);
      frame.info.origin.orientation.w = std::cos(yaw * .5);
      frame.data.resize(static_cast<std::size_t>(frame.info.width) * frame.info.height);
      for (auto & value : frame.data) {
        int v;
        if (!(input >> v) || v < -1 || v > 100) {throw std::runtime_error("invalid terrain cell");}
        value = static_cast<int8_t>(v);
      }
      const auto hard = std::count(frame.data.begin(), frame.data.end(), 100);
      for (std::size_t i = 0; i < filters.size(); ++i) {
        const auto output = filters[i].process(frame);
        const auto retained = std::count(output.data.begin(), output.data.end(), 100);
        const auto unknown = std::count(output.data.begin(), output.data.end(), -1);
        std::cout << count << ',' << mono << ',' << stamp << ',' << candidates[i] << ',' <<
          hard << ',' << retained << ',' << filters[i].suppressedCells() << ',' <<
          output.data.size() - retained - unknown << ',' << unknown << ',' << filters[i].resets() << '\n';
      }
      ++count;
    }
    if (!input.eof() || !count) {throw std::runtime_error("invalid or empty input");}
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
