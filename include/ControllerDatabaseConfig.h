#ifndef CONTROLLERDATABASECONFIG_H
#define CONTROLLERDATABASECONFIG_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <vector>

class ControllerDatabaseConfig {
   public:
      ControllerDatabaseConfig(const YAML::Node& config);
      ~ControllerDatabaseConfig();

   private:
      YAML::Node config;

};

#endif  // CONTROLLERDATABASECONFIG_H