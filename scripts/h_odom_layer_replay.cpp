// Cold-start recorded component replay: real rays/columns/store, not epoch or fusion acceptance.
#include "ats_rog_map/odom_layer_integration.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  try {
    if (argc != 4) {throw std::runtime_error("usage: h_odom_layer_replay columns.txt origins.txt timing.csv");}
    std::ifstream input(argv[1]), origins(argv[2]);
    std::ofstream timing(argv[3]);
    if (!input || !origins || !timing) {throw std::runtime_error("cannot open replay files");}
    int width, height;
    double resolution, ox, oy, xmin, ymin, xmax, ymax;
    ats_rog_map::GroundColumnConfig config;
    if (!(input >> width >> height >> resolution >> ox >> oy >> config.surface_max >>
      config.robot_height >> config.top_margin >> config.max_ground_slope >> xmin >> ymin >> xmax >> ymax))
    {throw std::runtime_error("invalid input header");}
    ats_rog_map::GroundColumnGrid columns(width, height, resolution, ox, oy, config);
    // Store policy defaults are explicit in the evidence metadata. This input
    // format is not a serialization of every effective ROS parameter.
    ats_rog_map::OdomLayerConfig lc;
    lc.width = width; lc.height = height; lc.resolution = resolution;
    lc.origin_x = ox; lc.origin_y = oy;
    ats_rog_map::OdomLayerStore layer(lc);
    std::cout << "stamp,column_x,column_y,column_state,layer_state\n" << std::setprecision(17);
    timing << "stamp,prepare_ms,ray_ms,ground_ms,evidence_ms,ray_voxels\n" << std::setprecision(17);
    const auto name = [](auto state) {
        const int value = static_cast<int>(state);
        return value == 0 ? "unknown" : (value == 1 ? "free" : "occupied");
      };
    double stamp, previous = 0;
    std::size_t frames = 0;
    while (input >> stamp) {
      std::size_t count;
      std::array<double, 3> seed, sensor;
      double sensor_stamp;
      if (!(input >> count >> seed[0] >> seed[1] >> seed[2]) ||
        !(origins >> sensor_stamp >> sensor[0] >> sensor[1] >> sensor[2]) ||
        !std::isfinite(stamp) || std::abs(sensor_stamp - stamp) > 1e-9 ||
        (frames && stamp <= previous))
      {throw std::runtime_error("invalid, missing or mismatched frame/origin");}
      std::vector<std::array<double, 3>> points(count);
      for (auto & p : points) {
        if (!(input >> p[0] >> p[1] >> p[2])) {throw std::runtime_error("truncated points");}
      }
      const auto stats = ats_rog_map::integrateOdomLayerFrame(layer, columns, points, sensor,
        seed, stamp, frames ? stamp - previous : 0.0);
      timing << stamp << ',' << stats.prepare_ms << ',' << stats.ray_ms << ',' <<
        stats.ground_ms << ',' << stats.evidence_ms << ',' << stats.ray_voxels << '\n';
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const double cx = ox + (x + 0.5) * resolution, cy = oy + (y + 0.5) * resolution;
          if (cx < xmin || cx > xmax || cy < ymin || cy > ymax) {continue;}
          std::cout << stamp << ',' << x << ',' << y << ',' << name(columns.state(x, y)) <<
            ',' << name(layer.cell(x, y)->state) << '\n';
        }
      }
      previous = stamp; ++frames;
    }
    std::string extra;
    if (!input.eof() || !frames || (origins >> extra)) {throw std::runtime_error("invalid or extra frames");}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
