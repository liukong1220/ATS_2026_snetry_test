// Offline component replay; deliberately excludes rays, epochs and map commits.
#include "ats_rog_map/ground_column.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using ats_rog_map::ColumnState;
using ats_rog_map::GroundColumnConfig;
using ats_rog_map::GroundColumnGrid;

void selfTest()
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  const auto check = [](bool ok) {if (!ok) {throw std::runtime_error("replay self-test failed");}};
  const auto classify = [&](std::vector<double> z) {
      return ats_rog_map::classifyGroundColumn(z, 3, 0.0, config);
    };
  check(classify({0.0, 0.247}) == ColumnState::kFree);
  check(classify({0.0, 0.237}) == ColumnState::kOccupied);
  check(classify({0.0, 0.239, 0.25}) == ColumnState::kOccupied);
  check(ats_rog_map::classifyGroundColumn({0.25}, 3, std::nullopt, config) ==
    ColumnState::kUnknown);
  GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  grid.observe({{0.05, 0.05, 0.25}}, {0.0, 0.0, 0.0});
  check(grid.state(5, 5) == ColumnState::kUnknown);
  grid.clear();
  grid.observe({{0.05, 0.05, 0.0}, {0.15, 0.05, 0.03}, {0.15, 0.05, 0.17}},
    {0.0, 0.0, 0.0});
  check(grid.state(6, 5) == ColumnState::kOccupied);
  std::cout << "6 component self-checks passed\n";
}

int main(int argc, char ** argv)
{
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {selfTest(); return 0;}
    if (argc != 2) {throw std::runtime_error("usage: h_ground_column_replay input.txt");}
    std::ifstream input(argv[1]);
    int width, height;
    double resolution, origin_x, origin_y, xmin, ymin, xmax, ymax;
    GroundColumnConfig config;
    if (!(input >> width >> height >> resolution >> origin_x >> origin_y >>
      config.surface_max >> config.robot_height >> config.top_margin >>
      config.max_ground_slope >> xmin >> ymin >> xmax >> ymax))
    {throw std::runtime_error("invalid input header");}
    GroundColumnGrid grid(width, height, resolution, origin_x, origin_y, config);
    std::cout << "stamp,column_x,column_y,ground_z,state\n" << std::setprecision(17);
    std::size_t count;
    double stamp;
    std::array<double, 3> seed;
    while (input >> stamp) {
      if (!(input >> count >> seed[0] >> seed[1] >> seed[2])) {
        throw std::runtime_error("truncated frame header");
      }
      std::vector<std::array<double, 3>> points(count);
      for (auto & point : points) {
        if (!(input >> point[0] >> point[1] >> point[2])) {
          throw std::runtime_error("truncated frame points");
        }
      }
      grid.observe(points, seed);
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const double cx = origin_x + (x + 0.5) * resolution;
          const double cy = origin_y + (y + 0.5) * resolution;
          if (cx < xmin || cx > xmax || cy < ymin || cy > ymax) {continue;}
          const auto ground = grid.ground(x, y);
          const auto state = grid.state(x, y);
          std::cout << stamp << ',' << x << ',' << y << ',';
          if (ground) {std::cout << *ground;}
          std::cout << ',' << (state == ColumnState::kFree ? "free" :
            state == ColumnState::kOccupied ? "occupied" : "unknown") << '\n';
        }
      }
    }
    if (!input.eof()) {throw std::runtime_error("invalid frame stream");}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
