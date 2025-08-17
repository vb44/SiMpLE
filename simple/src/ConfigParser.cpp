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
        voxelSizeMap_ = configFromYaml["voxelSizeMap"].as<double>();
        voxelSizeScan_ = configFromYaml["voxelSizeScan"].as<double>();
        convergenceTol_ = configFromYaml["convergenceTol"].as<double>();
        maxSensorRange_ = configFromYaml["maxSensorRange"].as<double>();
        minSensorRange_ = configFromYaml["minSensorRange"].as<double>();
        scanPath_ = configFromYaml["scanPath"].as<std::string>();
        outputFileName_ = configFromYaml["outputFileName"].as<std::string>();

        scanIntervalLoopClosure_ = configFromYaml["scanIntervalLoopClosure"].as<int>();
        scanIntervalPGO_ = configFromYaml["scanIntervalPGO"].as<int>();
        keyframeMeterGap_ = configFromYaml["keyframeMeterGap"].as<double>();
        keyframeDegGap_ = configFromYaml["keyframeDegGap"].as<double>();
        relinearizeThreshold_ = configFromYaml["relinearizeThreshold"].as<double>();
        relinearizeSkip_ = configFromYaml["relinearizeSkip"].as<double>();
        scDistThres_ = configFromYaml["scDistThres"].as<double>();
        scMaximumRadius_ = configFromYaml["scMaximumRadius"].as<double>(); 
        filterSize_ = configFromYaml["filterSize"].as<double>();
        mapVizFilterSize_ = configFromYaml["mapVizFilterSize"].as<double>();
        loopFitnessScoreThreshold_ = configFromYaml["loopFitnessScoreThreshold"].as<double>();

    } catch(const YAML::BadFile& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    }
    return 0;
}

bool ConfigParser::getKitti() const {
    return kitti_;
}

double ConfigParser::getSigma() const {
    return sigma_;
}

double ConfigParser::getVoxelSizeMap() const {
    return voxelSizeMap_;
}

double ConfigParser::getVoxelSizeScan() const {
    return voxelSizeScan_;
}

double ConfigParser::getConvergenceTol() const {
    return convergenceTol_;
}

double ConfigParser::getMaxSensorRange() const {
    return maxSensorRange_;
}

double ConfigParser::getMinSensorRange() const {
    return minSensorRange_;
}

const std::string ConfigParser::getScanPath() const {
    return scanPath_;
}

const std::string ConfigParser::getOutputFileName() const {
    return outputFileName_;
}

int ConfigParser::getScanIntervalLoopClosure() const {
    return scanIntervalLoopClosure_;
}

int ConfigParser::getScanIntervalPGO() const {
    return scanIntervalPGO_;
}

double ConfigParser::getKeyframeMeterGap() const {
    return keyframeMeterGap_;
}

double ConfigParser::getKeyframeDegGap() const {
    return keyframeDegGap_;
}

double ConfigParser::getRelinearizeThreshold() const {
    return relinearizeThreshold_;
}

double ConfigParser::getRelinearizeSkip() const {
    return relinearizeSkip_;
}

double ConfigParser::getScDistThres() const {
    return scDistThres_;
}

double ConfigParser::getScMaximumRadius() const {
    return scMaximumRadius_;
}

double ConfigParser::getFilterSize() const {
    return filterSize_;
}

double ConfigParser::getMapVizFilterSize() const {
    return mapVizFilterSize_;
}

double ConfigParser::getLoopFitnessScoreThreshold() const {
    return loopFitnessScoreThreshold_;
}