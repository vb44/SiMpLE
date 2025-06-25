#include "ConfigParser.hpp"

ConfigParser::ConfigParser(int argc, char** argv) {
    if (argc != EXPECTED_ARGUMENT_COUNT) {
        std::cerr << "Usage: ./simple config_file.yaml" << std::endl;
        exit(EXIT_FAILURE);
    }
    yamlFilePath_ = argv[1];
}

int ConfigParser::parseConfig() {
    try {
        YAML::Node configFromYaml = YAML::LoadFile(yamlFilePath_);
        kitti_ = configFromYaml["kitti"].as<bool>();
        sigma_ = configFromYaml["sigma"].as<double>();
        rMap_ = configFromYaml["rMap"].as<double>();
        rNew_ = configFromYaml["rNew"].as<double>();
        convergenceTol_ = configFromYaml["convergenceTol"].as<double>();
        maxSensorRange_ = configFromYaml["maxSensorRange"].as<double>();
        minSensorRange_ = configFromYaml["minSensorRange"].as<double>();
        scanPath_ = configFromYaml["scanPath"].as<std::string>();
        outputFileName_ = configFromYaml["outputFileName"].as<std::string>();

    } catch(const YAML::BadFile& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    }
    return 0;
}

const bool ConfigParser::getKitti() const {
    return kitti_;
}

const double ConfigParser::getSigma() const {
    return sigma_;
}

const double ConfigParser::getRMap() const {
    return rMap_;
}

const double ConfigParser::getRNew() const {
    return rNew_;
}

const double ConfigParser::getConvergenceTol() const {
    return convergenceTol_;
}

const double ConfigParser::getMaxSensorRange() const {
    return maxSensorRange_;
}

const double ConfigParser::getMinSensorRange() const {
    return minSensorRange_;
}

const std::string ConfigParser::getScanPath() const {
    return scanPath_;
}

const std::string ConfigParser::getOutputFileName() const {
    return outputFileName_;
}