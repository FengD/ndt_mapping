// The header file of the yaml generators

#ifndef _YAML_EDITOR_H_
#define _YAML_EDITOR_H_

#include <string>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

// The struct of the 6D pose
struct Calibration_Lidar{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};

// The yaml class
class CalibrationYaml{

  private:
    Calibration_Lidar calibrationLidar;

  public:
    // Constructor
    CalibrationYaml(){}
    // Destructor
    ~CalibrationYaml(){}
    // yaml writer for create the calibration file and for write the value in it
    void yamlWrite(std::string filePath, Calibration_Lidar cl){
      std::ofstream fout(filePath.c_str());
      YAML::Emitter out(fout);
      out << YAML::BeginMap;
      out << YAML::Key << "correction_x";
      out << YAML::Value << cl.x;
      out << YAML::Comment("given by hand if necessary");
      out << YAML::Key << "correction_y";
      out << YAML::Value << cl.y;
      out << YAML::Comment("given by hand if necessary");
      out << YAML::Key << "correction_z";
      out << YAML::Value << cl.z;
      out << YAML::Key << "correction_roll";
      out << YAML::Value << cl.roll;
      out << YAML::Key << "correction_pitch";
      out << YAML::Value << cl.pitch;
      out << YAML::Key << "correction_yaw";
      out << YAML::Value << cl.yaw;
      out << YAML::Comment("given by hand if necessary");
      out << YAML::EndMap;
    }
    // yaml reader for read the calibration yaml file
    void yamlRead(std::string filePath){
      YAML::Node yamlConfig = YAML::LoadFile(filePath);
      calibrationLidar.x = yamlConfig["correction_x"].as<float>();
      calibrationLidar.y = yamlConfig["correction_y"].as<float>();
      calibrationLidar.z = yamlConfig["correction_z"].as<float>();
      calibrationLidar.roll = yamlConfig["correction_roll"].as<float>() / 180.0 * M_PI;
      calibrationLidar.pitch = yamlConfig["correction_pitch"].as<float>() / 180.0 * M_PI;
      calibrationLidar.yaw = yamlConfig["correction_yaw"].as<float>() / 180.0 * M_PI;
    }
    // get the calibration result
    Calibration_Lidar getCalibrationLidar(){
      return calibrationLidar;
    }
};


#endif
