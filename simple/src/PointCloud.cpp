#include "PointCloud.hpp"

PointCloud::PointCloud(double maxSensorRange) :
      maxSensorRange2_(maxSensorRange * maxSensorRange) {
}

PointCloud::PointCloud(double voxelSize, double maxSensorRange, double minSensorRange, bool kitti) :
      voxelSize_(voxelSize),
      maxSensorRange2_(maxSensorRange * maxSensorRange),
      minSensorRange2_(minSensorRange * minSensorRange),
      kitti_(kitti) {
}

const std::vector<Eigen::Vector4d>& PointCloud::getPtCloud() const {
    return ptCloud_;
}
        
const NanoflannPointsContainer<double>& PointCloud::getPcForKdTree() const {
    return pcForKdTree_;
}

void PointCloud::addPoint(double x, double y, double z) {
    double normSquared = pow(x, 2) + pow(y, 2) + pow(z, 2);
    if ((normSquared > minSensorRange2_) && (normSquared < maxSensorRange2_)) {
      ptCloud_.push_back({x, y, z, 1});
    }
}

void PointCloud::readScan(std::string fileName) {
    ptCloud_.clear();

    std::ifstream file(fileName, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open file: " + fileName);
    }

    // Read the entire file into memory at once
    file.seekg(0, std::ios::end);
    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    if (fileSize % (sizeof(float) * NUM_COLUMNS_BIN) != 0) {
        throw std::runtime_error("File size does not match the .bin file structure!");
    }

    const size_t numPts = fileSize / (sizeof(float) * NUM_COLUMNS_BIN);
    std::vector<float> buffer(numPts * NUM_COLUMNS_BIN);
    file.read(reinterpret_cast<char*>(buffer.data()), fileSize);

    ptCloud_.reserve(numPts); // preallocate output vector

    for (size_t i = 0; i < buffer.size(); i += NUM_COLUMNS_BIN) {
        double x = static_cast<double>(buffer[i]);
        double y = static_cast<double>(buffer[i + 1]);
        double z = static_cast<double>(buffer[i + 2]);
        double norm2 = x * x + y * y + z * z;
        if (norm2 > minSensorRange2_ && norm2 < maxSensorRange2_) {
            ptCloud_.emplace_back(Eigen::Vector4d{x, y, z, 1.0});
        }
    }

    processPointCloud();
}


void PointCloud::processPointCloud() {
    // Check if the PointCloud needs to be corrected.
    // Apply the calibration factor as explained in IMLS-SLAM, CT-ICP, and KISS-ICP.
    if (kitti_) {
        correctKittiScan_();
    }

    // Subsample the point cloud.
    subsample_(ptCloud_, voxelSize_);
}

void PointCloud::correctKittiScan_() {
    // Adapted from KISS-ICP opensource code to correct the KITTI scans.
    constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
    
    tbb::parallel_for(
    tbb::blocked_range<int>(0, ptCloud_.size()),
    [&](tbb::blocked_range<int> r) {
        for (unsigned int i = r.begin(); i < r.end(); i++) {
            Eigen::Vector3d pt;
            Eigen::Vector3d ptCorrected;
            pt << ptCloud_[i](0), ptCloud_[i][1], ptCloud_[i][2];
            const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
            ptCorrected = Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
            ptCloud_[i] = {ptCorrected(0), ptCorrected(1), ptCorrected(2), 1};
        }
    });
}

void PointCloud::subsample_(std::vector<Eigen::Vector4d> &pts, double voxelSize)
{
    if (pts.empty()) return;

    const double invR = 1.0 / voxelSize;
    const double voxelSize2 = voxelSize * voxelSize;

    auto voxelKey = [&](int gx, int gy, int gz) -> int64_t {
        return (static_cast<int64_t>(gx) << 42)
             ^ (static_cast<int64_t>(gy) << 21)
             ^ static_cast<int64_t>(gz);
    };

    auto computeKey = [&](const Eigen::Vector4d &p) -> int64_t {
        const int gx = static_cast<int>(std::floor(p[0] * invR));
        const int gy = static_cast<int>(std::floor(p[1] * invR));
        const int gz = static_cast<int>(std::floor(p[2] * invR));
        return voxelKey(gx, gy, gz);
    };

    std::unordered_map<int64_t, std::vector<Eigen::Vector4d>> voxelMap;
    voxelMap.reserve(pts.size() / 2); // heurstic of expected number of voxels

    std::vector<Eigen::Vector4d> kept;
    kept.reserve(pts.size() / 5); // heurstic of expected number of voxels

    for (const auto &p : pts) {
        const int gx = static_cast<int>(std::floor(p[0] * invR));
        const int gy = static_cast<int>(std::floor(p[1] * invR));
        const int gz = static_cast<int>(std::floor(p[2] * invR));

        bool tooClose = false;

        // Check 3×3×3 neighboring voxels
        for (int dx = -1; dx <= 1 && !tooClose; ++dx) {
            for (int dy = -1; dy <= 1 && !tooClose; ++dy) {
                for (int dz = -1; dz <= 1 && !tooClose; ++dz) {
                    const int64_t key = voxelKey(gx + dx, gy + dy, gz + dz);
                    auto it = voxelMap.find(key);
                    if (it == voxelMap.end()) continue;
                    for (const auto &q : it->second) {
                        const double d2 = (p.head<3>() - q.head<3>()).squaredNorm();
                        if (d2 < voxelSize2) {
                            tooClose = true;
                            break;
                        }
                    }
                }
            }
        }

        if (!tooClose) {
            kept.push_back(p);
            const int64_t key = voxelKey(gx, gy, gz);
            voxelMap[key].push_back(p);
        }
    }

    // Deterministic ordering
    std::vector<std::pair<int64_t, Eigen::Vector4d>> sorted;
    sorted.reserve(kept.size());
    for (const auto &p : kept)
        sorted.emplace_back(computeKey(p), p);

    std::sort(sorted.begin(), sorted.end(),
              [](const auto &a, const auto &b){ return a.first < b.first; });

    pts.clear();
    pts.reserve(sorted.size());
    for (const auto &kv : sorted)
        pts.push_back(kv.second);
}

void PointCloud::convertToPointCloudKdTree_(std::vector<Eigen::Vector4d> &pts) {
    size_t pcLength = pts.size();
    pcForKdTree_.pts.clear();
    pcForKdTree_.pts.resize(pcLength);

    tbb::parallel_for(
    tbb::blocked_range<int>(0, pcLength),
    [&](tbb::blocked_range<int> r) { 
        for (size_t i = r.begin(); i < r.end(); i++) {
            pcForKdTree_.pts[i].x = pts[i](0);
            pcForKdTree_.pts[i].y = pts[i](1);
            pcForKdTree_.pts[i].z = pts[i](2);        
        } 
    });
}
