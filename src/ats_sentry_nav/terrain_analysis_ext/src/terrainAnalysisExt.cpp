 

#include <algorithm>
#include <limits>
#include <math.h>
#include <queue>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

double scanVoxelSize = 0.1;
double decayTime = 10.0;
double noDecayDis = 0;
double clearingDis = 30.0;
bool clearingCloud = false;
bool useSorting = false;
double quantileZ = 0.25;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double lowerBoundZ = -1.5;
double upperBoundZ = 1.0;
double disRatioZ = 0.1;
bool checkTerrainConn = true;
double terrainUnderVehicle = -0.75;
double terrainConnThre = 0.5;
double ceilingFilteringThre = 2.0;
double localTerrainMapRadius = 4.0;
double traversabilityObstacleHeightThre = 0.18;
double traversabilitySafeHeightThre = 0.08;
double traversabilityOccupancyRatioThre = 0.45;
double traversabilityGroundConfidenceThre = 0.55;
double traversabilityHeightWeight = 0.55;
double traversabilityOccupancyWeight = 0.30;
double traversabilityGroundWeight = 0.15;
double traversabilityDebugHeightDiffCap = 0.30;
double traversabilityDebugOccupancyRatioCap = 1.0;
int traversabilityMinPointCount = 3;
bool traversabilityUnknownAsOccupied = false;
bool publishTraversabilityDebugGrids = true;
double slopeGridMaxDeg = 45.0;
double slopeGentleDegThre = 8.0;
double slopeModerateDegThre = 15.0;
double slopeSteepDegThre = 25.0;
bool useSlopeAsObstacle = false;
double slopeObstacleDegThre = 28.0;

// terrain voxel parameters
float terrainVoxelSize = 2.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 41;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
constexpr int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// planar voxel parameters
float planarVoxelSize = 0.4;
const int planarVoxelWidth = 101;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
constexpr int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[kTerrainVoxelNum];

int terrainVoxelUpdateNum[kTerrainVoxelNum] = {0};
float terrainVoxelUpdateTime[kTerrainVoxelNum] = {0};
float planarVoxelElev[kPlanarVoxelNum] = {0};
int planarVoxelConn[kPlanarVoxelNum] = {0};
float planarVoxelMaxRelHeight[kPlanarVoxelNum] = {0};
float planarVoxelMinZ[kPlanarVoxelNum] = {0};
float planarVoxelMaxZ[kPlanarVoxelNum] = {0};
float planarVoxelHeightDiff[kPlanarVoxelNum] = {0};
float planarVoxelOccupancyRatio[kPlanarVoxelNum] = {0};
float planarVoxelGroundConfidence[kPlanarVoxelNum] = {0};
float planarVoxelSlopeDeg[kPlanarVoxelNum] = {0};
float planarVoxelSlopeBand[kPlanarVoxelNum] = {0};
int planarVoxelPointCount[kPlanarVoxelNum] = {0};
std::vector<float> planarPointElev[kPlanarVoxelNum];
std::queue<int> planarVoxelQueue;

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

// state estimation callback function
void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;
}

// registered laser scan callback function
void laserCloudHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
  laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();

  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));
    if (pointZ - vehicleZ > lowerBoundZ - disRatioZ * dis &&
        pointZ - vehicleZ < upperBoundZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// local terrain cloud callback function
void terrainCloudLocalHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloudLocal2) {
  terrainCloudLocal->clear();
  pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);
}

// joystick callback function
void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
  if (joy->buttons[5] > 0.5) {
    clearingCloud = true;
  }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
  clearingDis = dis->data;
  clearingCloud = true;
}

double clampUnit(double value) {
  return std::max(0.0, std::min(1.0, value));
}

int8_t normalizedToOccupancy(double value) {
  return static_cast<int8_t>(std::round(clampUnit(value) * 100.0));
}

// Convert a continuous slope angle into a few coarse bands so RViz can quickly
// show "gentle / moderate / steep" terrain without requiring custom messages.
double slopeDegToBand(double slope_deg) {
  if (slope_deg >= slopeSteepDegThre) {
    return 1.0;
  }
  if (slope_deg >= slopeModerateDegThre) {
    return 0.67;
  }
  if (slope_deg >= slopeGentleDegThre) {
    return 0.34;
  }
  return 0.0;
}

// Estimate the local slope from neighboring ground elevations on the 2D planar
// grid. This keeps V1 in a 2D navigation topology while still exposing 2.5D
// terrain semantics to later speed planning and obstacle policies.
void computeSlopeGrid() {
  const double resolution = std::max(static_cast<double>(planarVoxelSize), 1e-3);

  for (int indX = 0; indX < planarVoxelWidth; ++indX) {
    for (int indY = 0; indY < planarVoxelWidth; ++indY) {
      const int ind = planarVoxelWidth * indX + indY;
      const bool has_points =
          planarVoxelPointCount[ind] >= traversabilityMinPointCount;
      const bool is_connected = !checkTerrainConn || planarVoxelConn[ind] == 2;
      if (!has_points || !is_connected) {
        planarVoxelSlopeDeg[ind] = 0.0f;
        planarVoxelSlopeBand[ind] = 0.0f;
        continue;
      }

      const auto sample_elev = [&](int sx, int sy, float &elev) -> bool {
        if (sx < 0 || sx >= planarVoxelWidth || sy < 0 || sy >= planarVoxelWidth) {
          return false;
        }
        const int sample_ind = planarVoxelWidth * sx + sy;
        const bool sample_has_points =
            planarVoxelPointCount[sample_ind] >= traversabilityMinPointCount;
        const bool sample_is_connected =
            !checkTerrainConn || planarVoxelConn[sample_ind] == 2;
        if (!sample_has_points || !sample_is_connected) {
          return false;
        }
        elev = planarVoxelElev[sample_ind];
        return true;
      };

      float left = 0.0f;
      float right = 0.0f;
      float down = 0.0f;
      float up = 0.0f;
      const bool has_left = sample_elev(indX - 1, indY, left);
      const bool has_right = sample_elev(indX + 1, indY, right);
      const bool has_down = sample_elev(indX, indY - 1, down);
      const bool has_up = sample_elev(indX, indY + 1, up);

      double dzdx = 0.0;
      double dzdy = 0.0;
      bool valid_dx = false;
      bool valid_dy = false;

      if (has_left && has_right) {
        dzdx = (static_cast<double>(right) - static_cast<double>(left)) /
               (2.0 * resolution);
        valid_dx = true;
      } else if (has_right) {
        dzdx = (static_cast<double>(right) -
                static_cast<double>(planarVoxelElev[ind])) /
               resolution;
        valid_dx = true;
      } else if (has_left) {
        dzdx = (static_cast<double>(planarVoxelElev[ind]) -
                static_cast<double>(left)) /
               resolution;
        valid_dx = true;
      }

      if (has_down && has_up) {
        dzdy = (static_cast<double>(up) - static_cast<double>(down)) /
               (2.0 * resolution);
        valid_dy = true;
      } else if (has_up) {
        dzdy = (static_cast<double>(up) -
                static_cast<double>(planarVoxelElev[ind])) /
               resolution;
        valid_dy = true;
      } else if (has_down) {
        dzdy = (static_cast<double>(planarVoxelElev[ind]) -
                static_cast<double>(down)) /
               resolution;
        valid_dy = true;
      }

      if (!valid_dx && !valid_dy) {
        planarVoxelSlopeDeg[ind] = 0.0f;
        planarVoxelSlopeBand[ind] = 0.0f;
        continue;
      }

      const double gradient_norm = std::sqrt(dzdx * dzdx + dzdy * dzdy);
      const double slope_deg = std::atan(gradient_norm) * 180.0 / M_PI;
      planarVoxelSlopeDeg[ind] = static_cast<float>(slope_deg);
      planarVoxelSlopeBand[ind] = static_cast<float>(slopeDegToBand(slope_deg));
    }
  }
}

nav_msgs::msg::OccupancyGrid makePlanarGridMessage(const rclcpp::Time &stamp) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = "odom";
  grid.info.map_load_time = stamp;
  grid.info.resolution = planarVoxelSize;
  grid.info.width = planarVoxelWidth;
  grid.info.height = planarVoxelWidth;
  grid.info.origin.position.x =
      vehicleX - (planarVoxelHalfWidth + 0.5) * planarVoxelSize;
  grid.info.origin.position.y =
      vehicleY - (planarVoxelHalfWidth + 0.5) * planarVoxelSize;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(kPlanarVoxelNum, -1);
  return grid;
}

void publishScalarGrid(
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &publisher,
    const rclcpp::Time &stamp, const float *values, double scale_limit) {
  if (!publisher || scale_limit <= 0.0) {
    return;
  }

  nav_msgs::msg::OccupancyGrid grid = makePlanarGridMessage(stamp);
  for (int i = 0; i < kPlanarVoxelNum; ++i) {
    const bool has_points = planarVoxelPointCount[i] >= traversabilityMinPointCount;
    if (!has_points) {
      continue;
    }
    grid.data[static_cast<std::size_t>(i)] =
        normalizedToOccupancy(static_cast<double>(values[i]) / scale_limit);
  }

  publisher->publish(grid);
}

void publishTraversabilityGrid(
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &height_diff_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &occupancy_ratio_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &ground_confidence_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &slope_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &slope_band_publisher,
    const rclcpp::Time &stamp) {
  nav_msgs::msg::OccupancyGrid grid = makePlanarGridMessage(stamp);

  const double height_range = std::max(
      traversabilityObstacleHeightThre - traversabilitySafeHeightThre, 1e-3);
  const double weight_sum =
      std::max(traversabilityHeightWeight + traversabilityOccupancyWeight +
                   traversabilityGroundWeight,
               1e-6);

  for (int i = 0; i < kPlanarVoxelNum; ++i) {
    const bool has_points = planarVoxelPointCount[i] >= traversabilityMinPointCount;
    const bool is_connected = !checkTerrainConn || planarVoxelConn[i] == 2;
    if (!has_points) {
      grid.data[static_cast<std::size_t>(i)] =
          traversabilityUnknownAsOccupied ? 100 : -1;
      continue;
    }

    const double point_confidence = clampUnit(
        static_cast<double>(planarVoxelPointCount[i]) /
        static_cast<double>(std::max(traversabilityMinPointCount * 2, 1)));
    const double connectivity_confidence =
        !checkTerrainConn ? 1.0 : (is_connected ? 1.0 : 0.0);
    const double ground_confidence =
        0.6 * point_confidence + 0.4 * connectivity_confidence;
    planarVoxelGroundConfidence[i] = static_cast<float>(ground_confidence);

    const double height_metric = std::max(
        static_cast<double>(planarVoxelHeightDiff[i]),
        static_cast<double>(planarVoxelMaxRelHeight[i]));
    const double height_score = clampUnit(
        (height_metric - traversabilitySafeHeightThre) / height_range);
    const double occupancy_activation = clampUnit(
        (height_metric - 0.5 * traversabilitySafeHeightThre) /
        std::max(traversabilityObstacleHeightThre - 0.5 * traversabilitySafeHeightThre,
                 1e-3));
    const double occupancy_score =
        occupancy_activation *
        clampUnit(planarVoxelOccupancyRatio[i] /
                  std::max(traversabilityOccupancyRatioThre, 1e-3));
    const double ground_penalty = clampUnit(
        (traversabilityGroundConfidenceThre - ground_confidence) /
        std::max(traversabilityGroundConfidenceThre, 1e-3));
    const double slope_deg = static_cast<double>(planarVoxelSlopeDeg[i]);
    const bool slope_blocked =
        useSlopeAsObstacle && slope_deg >= slopeObstacleDegThre;

    // V1 default: slope is published as terrain semantics for later velocity
    // adaptation. If `useSlopeAsObstacle` is enabled, steep cells are upgraded
    // to hard obstacles here so the 2D planner will route around them directly.
    double combined_score =
        (traversabilityHeightWeight * height_score +
         traversabilityOccupancyWeight * occupancy_score +
         traversabilityGroundWeight * ground_penalty) /
        weight_sum;
    if (!is_connected || height_metric >= traversabilityObstacleHeightThre ||
        slope_blocked) {
      combined_score = 1.0;
    }

    grid.data[static_cast<std::size_t>(i)] =
        normalizedToOccupancy(combined_score);
  }

  publisher->publish(grid);
  publishScalarGrid(slope_publisher, stamp, planarVoxelSlopeDeg,
                    std::max(slopeGridMaxDeg, 1e-3));
  publishScalarGrid(slope_band_publisher, stamp, planarVoxelSlopeBand, 1.0);
  if (publishTraversabilityDebugGrids) {
    publishScalarGrid(height_diff_publisher, stamp, planarVoxelHeightDiff,
                      traversabilityDebugHeightDiffCap);
    publishScalarGrid(occupancy_ratio_publisher, stamp, planarVoxelOccupancyRatio,
                      traversabilityDebugOccupancyRatioCap);
    publishScalarGrid(ground_confidence_publisher, stamp,
                      planarVoxelGroundConfidence, 1.0);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("terrainAnalysisExt");

  nh->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
  nh->declare_parameter<double>("decayTime", decayTime);
  nh->declare_parameter<double>("noDecayDis", noDecayDis);
  nh->declare_parameter<double>("clearingDis", clearingDis);
  nh->declare_parameter<bool>("useSorting", useSorting);
  nh->declare_parameter<double>("quantileZ", quantileZ);
  nh->declare_parameter<double>("vehicleHeight", vehicleHeight);
  nh->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->declare_parameter<double>("lowerBoundZ", lowerBoundZ);
  nh->declare_parameter<double>("upperBoundZ", upperBoundZ);
  nh->declare_parameter<double>("disRatioZ", disRatioZ);
  nh->declare_parameter<bool>("checkTerrainConn", checkTerrainConn);
  nh->declare_parameter<double>("terrainUnderVehicle", terrainUnderVehicle);
  nh->declare_parameter<double>("terrainConnThre", terrainConnThre);
  nh->declare_parameter<double>("ceilingFilteringThre", ceilingFilteringThre);
  nh->declare_parameter<double>("localTerrainMapRadius", localTerrainMapRadius);
  nh->declare_parameter<double>("traversabilityObstacleHeightThre",
                                traversabilityObstacleHeightThre);
  nh->declare_parameter<double>("traversabilitySafeHeightThre",
                                traversabilitySafeHeightThre);
  nh->declare_parameter<double>("traversabilityOccupancyRatioThre",
                                traversabilityOccupancyRatioThre);
  nh->declare_parameter<double>("traversabilityGroundConfidenceThre",
                                traversabilityGroundConfidenceThre);
  nh->declare_parameter<double>("traversabilityHeightWeight",
                                traversabilityHeightWeight);
  nh->declare_parameter<double>("traversabilityOccupancyWeight",
                                traversabilityOccupancyWeight);
  nh->declare_parameter<double>("traversabilityGroundWeight",
                                traversabilityGroundWeight);
  nh->declare_parameter<double>("traversabilityDebugHeightDiffCap",
                                traversabilityDebugHeightDiffCap);
  nh->declare_parameter<double>("traversabilityDebugOccupancyRatioCap",
                                traversabilityDebugOccupancyRatioCap);
  nh->declare_parameter<int>("traversabilityMinPointCount",
                             traversabilityMinPointCount);
  nh->declare_parameter<bool>("traversabilityUnknownAsOccupied",
                              traversabilityUnknownAsOccupied);
  nh->declare_parameter<bool>("publishTraversabilityDebugGrids",
                              publishTraversabilityDebugGrids);
  nh->declare_parameter<double>("slopeGridMaxDeg", slopeGridMaxDeg);
  nh->declare_parameter<double>("slopeGentleDegThre", slopeGentleDegThre);
  nh->declare_parameter<double>("slopeModerateDegThre", slopeModerateDegThre);
  nh->declare_parameter<double>("slopeSteepDegThre", slopeSteepDegThre);
  nh->declare_parameter<bool>("useSlopeAsObstacle", useSlopeAsObstacle);
  nh->declare_parameter<double>("slopeObstacleDegThre", slopeObstacleDegThre);

  nh->get_parameter("scanVoxelSize", scanVoxelSize);
  nh->get_parameter("decayTime", decayTime);
  nh->get_parameter("noDecayDis", noDecayDis);
  nh->get_parameter("clearingDis", clearingDis);
  nh->get_parameter("useSorting", useSorting);
  nh->get_parameter("quantileZ", quantileZ);
  nh->get_parameter("vehicleHeight", vehicleHeight);
  nh->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->get_parameter("lowerBoundZ", lowerBoundZ);
  nh->get_parameter("upperBoundZ", upperBoundZ);
  nh->get_parameter("disRatioZ", disRatioZ);
  nh->get_parameter("checkTerrainConn", checkTerrainConn);
  nh->get_parameter("terrainUnderVehicle", terrainUnderVehicle);
  nh->get_parameter("terrainConnThre", terrainConnThre);
  nh->get_parameter("ceilingFilteringThre", ceilingFilteringThre);
  nh->get_parameter("localTerrainMapRadius", localTerrainMapRadius);
  nh->get_parameter("traversabilityObstacleHeightThre",
                    traversabilityObstacleHeightThre);
  nh->get_parameter("traversabilitySafeHeightThre", traversabilitySafeHeightThre);
  nh->get_parameter("traversabilityOccupancyRatioThre",
                    traversabilityOccupancyRatioThre);
  nh->get_parameter("traversabilityGroundConfidenceThre",
                    traversabilityGroundConfidenceThre);
  nh->get_parameter("traversabilityHeightWeight", traversabilityHeightWeight);
  nh->get_parameter("traversabilityOccupancyWeight",
                    traversabilityOccupancyWeight);
  nh->get_parameter("traversabilityGroundWeight", traversabilityGroundWeight);
  nh->get_parameter("traversabilityDebugHeightDiffCap",
                    traversabilityDebugHeightDiffCap);
  nh->get_parameter("traversabilityDebugOccupancyRatioCap",
                    traversabilityDebugOccupancyRatioCap);
  nh->get_parameter("traversabilityMinPointCount",
                    traversabilityMinPointCount);
  nh->get_parameter("traversabilityUnknownAsOccupied",
                    traversabilityUnknownAsOccupied);
  nh->get_parameter("publishTraversabilityDebugGrids",
                    publishTraversabilityDebugGrids);
  nh->get_parameter("slopeGridMaxDeg", slopeGridMaxDeg);
  nh->get_parameter("slopeGentleDegThre", slopeGentleDegThre);
  nh->get_parameter("slopeModerateDegThre", slopeModerateDegThre);
  nh->get_parameter("slopeSteepDegThre", slopeSteepDegThre);
  nh->get_parameter("useSlopeAsObstacle", useSlopeAsObstacle);
  nh->get_parameter("slopeObstacleDegThre", slopeObstacleDegThre);

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      "registered_scan", 5, laserCloudHandler);

  auto subJoystick =
      nh->create_subscription<sensor_msgs::msg::Joy>("joy", 5, joystickHandler);

  auto subClearing = nh->create_subscription<std_msgs::msg::Float32>(
      "cloud_clearing", 5, clearingHandler);

  auto subTerrainCloudLocal =
      nh->create_subscription<sensor_msgs::msg::PointCloud2>(
          "terrain_map", 2, terrainCloudLocalHandler);

  auto pubTerrainCloud =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map_ext", 2);
  auto pubTraversabilityGrid =
      nh->create_publisher<nav_msgs::msg::OccupancyGrid>("traversability_grid", 2);
  auto pubTraversabilityHeightDiffGrid =
      nh->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "traversability_height_diff_grid", 2);
  auto pubTraversabilityOccupancyRatioGrid =
      nh->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "traversability_occupancy_ratio_grid", 2);
  auto pubTraversabilityGroundConfidenceGrid =
      nh->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "traversability_ground_confidence_grid", 2);
  auto pubTraversabilitySlopeGrid =
      nh->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "traversability_slope_grid", 2);
  auto pubTraversabilitySlopeBandGrid =
      nh->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "traversability_slope_band_grid", 2);

  for (int i = 0; i < kTerrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (newlaserCloud) {
      newlaserCloud = false;

      // terrain voxel roll over
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // stack registered laser scans
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = laserCloudCrop->points[i];

        int indX =
            static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2) /
                             terrainVoxelSize) +
            terrainVoxelHalfWidth;
        int indY =
            static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) /
                             terrainVoxelSize) +
            terrainVoxelHalfWidth;

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }

      for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                voxelTimeUpdateThre ||
            clearingCloud) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[ind];

          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz);

          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++) {
            point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
                point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.intensity <
                     decayTime ||
                 dis < noDecayDis) &&
                !(dis < clearingDis && clearingCloud)) {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 10;
           indX <= terrainVoxelHalfWidth + 10; indX++) {
        for (int indY = terrainVoxelHalfWidth - 10;
             indY <= terrainVoxelHalfWidth + 10; indY++) {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // Reset all per-cell semantic statistics, then rebuild them from the
      // latest aggregated terrain cloud.
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        planarVoxelElev[i] = 0;
        planarVoxelConn[i] = 0;
        planarVoxelMaxRelHeight[i] = 0.0f;
        planarVoxelMinZ[i] = std::numeric_limits<float>::infinity();
        planarVoxelMaxZ[i] = -std::numeric_limits<float>::infinity();
        planarVoxelHeightDiff[i] = 0.0f;
        planarVoxelOccupancyRatio[i] = 0.0f;
        planarVoxelGroundConfidence[i] = 0.0f;
        planarVoxelSlopeDeg[i] = 0.0f;
        planarVoxelSlopeBand[i] = 0.0f;
        planarVoxelPointCount[i] = 0;
        planarPointElev[i].clear();
      }

      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {
          int indX =
              static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;
          int indY =
              static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                    .push_back(point.z);
              }
            }
          }
        }
      }

      // Estimate ground elevation by a low quantile so sparse obstacle tops do
      // not easily drag the local ground plane upward.
      if (useSorting) {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());

            int quantileID = static_cast<int>(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            planarVoxelElev[i] = planarPointElev[i][quantileID];
          }
        }
      } else {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++) {
              if (planarPointElev[i][j] < minZ) {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1) {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      // Revisit raw points and accumulate per-cell vertical extent statistics.
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {
          int indX =
              static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;
          int indY =
              static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            const int ind = planarVoxelWidth * indX + indY;
            const float relHeight = point.z - planarVoxelElev[ind];
            if (relHeight >= 0.0f) {
              planarVoxelMaxRelHeight[ind] =
                  std::max(planarVoxelMaxRelHeight[ind], relHeight);
            }
            planarVoxelMinZ[ind] = std::min(planarVoxelMinZ[ind], point.z);
            planarVoxelMaxZ[ind] = std::max(planarVoxelMaxZ[ind], point.z);
            planarVoxelPointCount[ind]++;
          }
        }
      }

      // Convert raw point count and vertical spread into a simple occupancy
      // ratio. This is still a 2D cost input, not a full 3D occupancy model.
      for (int i = 0; i < kPlanarVoxelNum; ++i) {
        if (planarVoxelPointCount[i] <= 0 ||
            !std::isfinite(planarVoxelMinZ[i]) ||
            !std::isfinite(planarVoxelMaxZ[i])) {
          continue;
        }

        planarVoxelHeightDiff[i] =
            std::max(0.0f, planarVoxelMaxZ[i] - planarVoxelMinZ[i]);
        const double height_metric = std::max(
            static_cast<double>(planarVoxelHeightDiff[i]),
            static_cast<double>(planarVoxelMaxRelHeight[i]));
        if (height_metric <= 0.5 * traversabilitySafeHeightThre) {
          planarVoxelOccupancyRatio[i] = 0.0f;
          continue;
        }

        const double vertical_span =
            std::max(height_metric, static_cast<double>(scanVoxelSize));
        const double occupancy_ratio = clampUnit(
            ((static_cast<double>(planarVoxelPointCount[i]) + 1.0) *
             static_cast<double>(scanVoxelSize)) /
            vertical_span);
        planarVoxelOccupancyRatio[i] = static_cast<float>(occupancy_ratio);
      }

      // Optional connectivity flood-fill to reject disconnected ceiling-like
      // surfaces and preserve only terrain connected to the robot vicinity.
      if (checkTerrainConn) {
        int ind =
            planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
        if (planarPointElev[ind].size() == 0)
          planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;

        planarVoxelQueue.push(ind);
        planarVoxelConn[ind] = 1;
        while (!planarVoxelQueue.empty()) {
          int front = planarVoxelQueue.front();
          planarVoxelConn[front] = 2;
          planarVoxelQueue.pop();

          int indX = static_cast<int>(front / planarVoxelWidth);
          int indY = front % planarVoxelWidth;
          for (int dX = -10; dX <= 10; dX++) {
            for (int dY = -10; dY <= 10; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                ind = planarVoxelWidth * (indX + dX) + indY + dY;
                if (planarVoxelConn[ind] == 0 &&
                    planarPointElev[ind].size() > 0) {
                  if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) <
                      terrainConnThre) {
                    planarVoxelQueue.push(ind);
                    planarVoxelConn[ind] = 1;
                  } else if (fabs(planarVoxelElev[front] -
                                  planarVoxelElev[ind]) >
                             ceilingFilteringThre) {
                    planarVoxelConn[ind] = -1;
                  }
                }
              }
            }
          }
        }
      }

      // Compute slope only after ground elevation and connectivity have been
      // stabilized, so invalid neighbors do not contaminate the gradient.
      computeSlopeGrid();

      // compute terrain map beyond localTerrainMapRadius
      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
            dis > localTerrainMapRadius) {
          int indX =
              static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;
          int indY =
              static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            int ind = planarVoxelWidth * indX + indY;
            float disZ = fabs(point.z - planarVoxelElev[ind]);
            if (disZ < vehicleHeight &&
                (planarVoxelConn[ind] == 2 || !checkTerrainConn)) {
              terrainCloudElev->push_back(point);
              terrainCloudElev->points[terrainCloudElevSize].x = point.x;
              terrainCloudElev->points[terrainCloudElevSize].y = point.y;
              terrainCloudElev->points[terrainCloudElevSize].z = point.z;
              terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

              terrainCloudElevSize++;
            }
          }
        }
      }

      // merge in local terrain map within localTerrainMapRadius
      int terrainCloudLocalSize = terrainCloudLocal->points.size();
      for (int i = 0; i < terrainCloudLocalSize; i++) {
        point = terrainCloudLocal->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (dis <= localTerrainMapRadius) {
          terrainCloudElev->push_back(point);
        }
      }

      clearingCloud = false;

      // publish points with elevation
      sensor_msgs::msg::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      terrainCloud2.header.stamp =
          rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
      terrainCloud2.header.frame_id = "odom";
      pubTerrainCloud->publish(terrainCloud2);
      publishTraversabilityGrid(pubTraversabilityGrid,
                                pubTraversabilityHeightDiffGrid,
                                pubTraversabilityOccupancyRatioGrid,
                                pubTraversabilityGroundConfidenceGrid,
                                pubTraversabilitySlopeGrid,
                                pubTraversabilitySlopeBandGrid,
                                terrainCloud2.header.stamp);
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
