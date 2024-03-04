#include "ConfigParser.hpp"

ConfigParser::ConfigParser(int argc, char** yamlFilePath)
{
    if (argc != 2)
    {
        std::cerr << "Usage: ./simple config_file.yaml" << std::endl;
        exit(EXIT_FAILURE);
    }
    yamlFilePath_ = yamlFilePath[1];
}

ConfigParser::~ConfigParser()
{
}

int ConfigParser::parseConfig()
{
    try {
        YAML::Node configFromYaml = YAML::LoadFile(yamlFilePath_);
        
        verbose = configFromYaml["verbose"].as<bool>();
        kitti = configFromYaml["kitti"].as<bool>();
        sigma = configFromYaml["sigma"].as<double>();
        rMap = configFromYaml["rMap"].as<double>();
        rNew = configFromYaml["rNew"].as<double>();
        convergenceTol = configFromYaml["convergenceTol"].as<double>();
        maxSensorRange = configFromYaml["maxSensorRange"].as<double>();
        minSensorRange = configFromYaml["minSensorRange"].as<double>();
        scanPath = configFromYaml["scanPath"].as<std::string>();
        outputFileName = configFromYaml["outputFileName"].as<std::string>();

    } catch(const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    }
    return 0;
}