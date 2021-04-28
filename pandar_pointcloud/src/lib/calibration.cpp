#include "pandar_pointcloud/calibration.hpp"
#include <fstream>
#include <sstream>

namespace pandar_pointcloud
{
Calibration::Calibration()
{
}

int Calibration::loadFile(const std::string& calibration_file)
{
  std::ifstream ifs(calibration_file);
  if (!ifs) {
    return -1;
  }

  std::string header;
  std::getline(ifs, header);

  char sep;
  int laser_id;
  float elevation;
  float azimuth;
  while (!ifs.eof()) {
    ifs >> laser_id >> sep >> elevation >> sep >> azimuth;
    elev_angle_map[laser_id - 1] = elevation;
    azimuth_offset_map[laser_id - 1] = azimuth;
  }
  ifs.close();
  return 0;
}

int Calibration::loadContent(const std::string& calibration_content)
{
  std::stringstream ss;
  ss << calibration_content;

  std::string header;
  std::getline(ss, header);

  char sep;
  int laser_id;
  float elevation;
  float azimuth;
  while (!ss.eof()) {
    ss >> laser_id >> sep >> elevation >> sep >> azimuth;
    elev_angle_map[laser_id - 1] = elevation;
    azimuth_offset_map[laser_id - 1] = azimuth;
  }
  return 0;
}

int Calibration::saveFile(const std::string& calibration_file)
{
  std::ofstream ofs(calibration_file);
  if (!ofs) {
    return -1;
  }
  ofs << "Laser id,Elevation,Azimuth" << std::endl;
  for (const auto& pair : elev_angle_map) {
    int laser_id = pair.first + 1;
    float elevation = pair.second;
    float azimuth = azimuth_offset_map[pair.first];
    ofs << laser_id << "," << elevation << "," << azimuth << std::endl;
  }
  ofs.close();

  return 0;
}
}  // namespace pandar_pointcloud