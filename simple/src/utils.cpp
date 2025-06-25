#include "utils.hpp"

namespace utils {

    Eigen::Matrix4d homogeneous(double roll, double pitch, double yaw, double x, double y, double z) {
        Eigen::Matrix4d T;
        T.setZero();
        T(0,0) = cos(yaw) * cos(pitch);
        T(0,1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
        T(0,2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
        T(0,3) = x;
        T(1,0) = sin(yaw) * cos(pitch);
        T(1,1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
        T(1,2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
        T(1,3) = y;
        T(2,0) = -sin(pitch);
        T(2,1) = cos(pitch) * sin(roll);
        T(2,2) = cos(pitch) * cos(roll);
        T(2,3) = z;
        T(3,3) = 1;
        return T;
    }

    std::vector<double> hom2rpyxyz(Eigen::Matrix4d T) {
        double roll = atan2(T(2,1), T(2,2));
        double pitch = asin(-T(2,0));
        double yaw = atan2(T(1,0), T(0,0));
        double x = T(0,3);
        double y = T(1,3);
        double z = T(2,3);
        std::vector<double> result = {roll, pitch, yaw, x, y, z};
        return result;
    }

    bool compareStrings(std::string a, std::string b) {
        std::string delimiterStart = "/";
        std::string delimiterEnd = ".bin";
        std::string aNum = a.substr(a.find_last_of(delimiterStart) + delimiterStart.size(), a.size());
        std::string bNum = b.substr(b.find_last_of(delimiterStart) + delimiterStart.size(), b.size());
        aNum = aNum.substr(0, aNum.find(delimiterEnd));
        bNum = bNum.substr(0, bNum.find(delimiterEnd));
        return stol(aNum) < stol(bNum);
    }

    void writeResults(ConfigParser &config, std::vector<std::vector<double> > poseEstimates,
                      std::string outputFileName, double avgTimePerScan) {
        // Write the results to file
        std::ofstream outputResultsFile(outputFileName+".txt");
        for (unsigned int i = 0; i < poseEstimates.size(); i++) {
            Eigen::Matrix4d res = homogeneous(poseEstimates[i][0], poseEstimates[i][1], poseEstimates[i][2],
                                              poseEstimates[i][3], poseEstimates[i][4], poseEstimates[i][5]);
            outputResultsFile << res(0,0) << " " << res(0,1) << " " << res(0,2) << " " << res(0,3) << " "
                              << res(1,0) << " " << res(1,1) << " " << res(1,2) << " " << res(1,3) << " "
                              << res(2,0) << " " << res(2,1) << " " << res(2,2) << " " << res(2,3) << std::endl;
        }
        outputResultsFile.close();
        std::time_t timeNow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        // Write the config to file
        std::string outputConfigFileName = outputFileName+"_config.txt";
        std::ofstream outputConfigFile(outputConfigFileName);
        outputConfigFile << "% computation finised at : " << std::ctime(&timeNow)
                         << std::endl
                         << "scansFolderPath        = " << "\"" <<  config.getScanPath()  << "\"" << ";" << std::endl
                         << "sigma                  = " << config.getSigma() << ";" << " % [m]" << std::endl
                         << "rMap                   = " << config.getRMap()  << ";" << " % [m]" << std::endl
                         << "rNew                   = " << config.getRNew()  << ";" << " % [m]" << std::endl
                         << "convergenceTolerance   = " << config.getConvergenceTol() << ";" << std::endl
                         << "maxSensorRange         = " << config.getMaxSensorRange() << ";" << " % [m]" << std::endl
                         << "minSensorRange         = " << config.getMinSensorRange() << ";" << " % [m]" << std::endl
                         << "outputFileName         = " << "\"" << config.getOutputFileName() << "\"" << ";" << std::endl
                         << "outputConfigFileName   = " << "\"" << outputConfigFileName << "\"" << ";" << std::endl
                         << "avg_time_per_scan      = " << avgTimePerScan << ";" << " % [ms]" << std::endl;
        outputConfigFile.close();
    }

    void printProgress(double percentage) {
        // code from https://
        // stackoverflow.com/questions/14539867/how-to-display-a-progress-indicator-in-pure-c-c-cout-printf
        int progressBarWidth = 60;
        char progressBarString[] = "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
        int val = (int) (percentage * 100);
        int lpad = (int) (percentage * progressBarWidth);
        int rpad = progressBarWidth - lpad;
        printf("\r%3d%% [%.*s%*s]", val, lpad, progressBarString, rpad, "");
        fflush(stdout);
    }
}