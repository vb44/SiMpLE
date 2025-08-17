#include "PoseGraphOptimization.hpp"

PoseGraphOptimization::PoseGraphOptimization(std::ofstream& logFile, double keyframeMeterGap, double keyframeDegGap,
                                             double scDistThres, double scMaximumRadius,
                                             double relinearizeThreshold, double relinearizeSkip,
                                             double filter_size, double mapVizFilterSize, double loopFitnessScoreThreshold, std::string saveDirectory)
    : logFile_(logFile),
      keyframeMeterGap_(keyframeMeterGap),
      loopFitnessScoreThreshold_(loopFitnessScoreThreshold),
      laserCloudFullRes(new pcl::PointCloud<PointType>()),
      laserCloudMapAfterPGO(new pcl::PointCloud<PointType>()),
      laserCloudMapPGO(new pcl::PointCloud<PointType>()) {
    
    // Default parameters
    translationAccumulated = 1000000.0; // large value means must add the first given frame.
    rotaionAccumulated = 1000000.0; // large value means must add the first given frame.
    isNowKeyFrame = false;
    timeLaserOdometry = 0.0;
    timeLaser = 0.0;
    recentIdxUpdated = 0;
    gtSAMgraphMade = false;
    laserCloudMapPGORedraw = true;
    recentOptimizedX = 0.0;
    recentOptimizedY = 0.0;

    // save directories 

    scManager.setLogFile(logFile_);
    pgKITTIformat_ = saveDirectory + "_pgo_poses.txt";
    odomKITTIformat_ = saveDirectory + "_odom_poses.txt";
    // // pgG2oSaveStream = std::fstream(save_directory + "singlesession_posegraph.g2o", std::fstream::out);
    // pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out); 
    // pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

    // pgScansDirectory = save_directory + "Scans/";
    // auto unused = system((std::string("exec rm -r ") + pgScansDirectory).c_str());
    // unused = system((std::string("mkdir -p ") + pgScansDirectory).c_str());

    // pgSCDsDirectory = save_directory + "SCDs/"; // SCD: scan context descriptor 
    // unused = system((std::string("exec rm -r ") + pgSCDsDirectory).c_str());
    // unused = system((std::string("mkdir -p ") + pgSCDsDirectory).c_str());

    // system params  
    keyframeRadGap_ = deg2rad(keyframeDegGap);
	
    ISAM2Params parameters;
    parameters.relinearizeThreshold = relinearizeThreshold;
    parameters.relinearizeSkip = relinearizeSkip;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

	
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

}

// TODO: Move to utils
double PoseGraphOptimization::rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

// TODO: Move to utils
double PoseGraphOptimization::deg2rad(double degrees) { 
  return degrees * M_PI / 180.0;
}
 
std::string PoseGraphOptimization::padZeros(int val, int num_digits) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

std::string PoseGraphOptimization::getVertexStr(const int _node_idx, const gtsam::Pose3& _Pose) {
    gtsam::Point3 t = _Pose.translation();
    gtsam::Rot3 R = _Pose.rotation();

    std::string curVertexInfo {
        "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    return curVertexInfo;
}

void PoseGraphOptimization::writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, std::vector<std::string>& edges_str) {
    gtsam::Point3 t = _relPose.translation();
    gtsam::Rot3 R = _relPose.rotation();

    std::string curEdgeInfo {
        "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    edges_str.emplace_back(curEdgeInfo);
}

void PoseGraphOptimization::saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter) {
    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open()) {
        file << matrix.format(the_format);
        file.close();
    }
}

gtsam::Pose3 PoseGraphOptimization::Pose6DtoGTSAMPose3(const Pose6D& p) {
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
}

// void PoseGraphOptimization::saveGTSAMgraphG2oFormat(const gtsam::Values& _estimates) {
//     // save pose graph (runs when programe is closing)
//     std::cerr << "Saving the posegraph ..." << std::endl; // giseop

//     pgG2oSaveStream = std::fstream(save_directory + "singlesession_posegraph.g2o", std::fstream::out);

//     int pose_idx = 0;
//     for(const auto& _pose6d: keyframePoses) {
//         gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);    
//         pgG2oSaveStream << getVertexStr(pose_idx, pose) << std::endl;
//         pose_idx++;
//     }
//     for(auto& _line: edges_str) {
//         pgG2oSaveStream << _line << std::endl;
//     }

//     pgG2oSaveStream.close();
// }

void PoseGraphOptimization::saveOdometryVerticesKITTIformat(std::string _filename) {
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void PoseGraphOptimization::saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename) {
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void PoseGraphOptimization::laserOdometryHandler(const Pose6D laserOdometry) {
	odometryBuf.push(laserOdometry);
}

void PoseGraphOptimization::laserCloudFullResHandler(const pcl::PointCloud<PointType>::Ptr laserCloudFullRes) {
	fullResBuf.push(laserCloudFullRes);
}

void PoseGraphOptimization::initNoises(void) {
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D PoseGraphOptimization::diffTransformation(const Pose6D& _p1, const Pose6D& _p2) {
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr PoseGraphOptimization::local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 12;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void PoseGraphOptimization::updatePoses(void) {
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++) {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }

    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

} // updatePoses

void PoseGraphOptimization::runISAM2opt(void) {
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr PoseGraphOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 12; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void PoseGraphOptimization::loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx) {
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i; 
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}


// TODO: Replace this with SiMpLE later?
std::optional<gtsam::Pose3> PoseGraphOptimization::doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx ) {
    // parse pointclouds
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold_) {
        logFile_ << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold_ << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        logFile_ << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold_ << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
}

void PoseGraphOptimization::process_pg() {
    while ( !odometryBuf.empty() && !fullResBuf.empty() )
    {
        //
        // pop and check keyframe is or not  
        // 
        while (!odometryBuf.empty() && odometryBufTime.front() < fullResBufTime.front())
            odometryBuf.pop();
        if (odometryBuf.empty()) {
            break;
        }

        // Time equal check
        timeLaserOdometry = odometryBufTime.front();
        timeLaser = fullResBufTime.front();
        // TODO

        laserCloudFullRes->clear();
        pcl::PointCloud<PointType>::Ptr thisKeyFrame = fullResBuf.front();
        fullResBuf.pop();

        Pose6D pose_curr = odometryBuf.front();
        odometryBuf.pop();

        //
        // Early reject by counting local delta movement (for equi-spereated kf drop)
        // 
        odom_pose_prev = odom_pose_curr;
        odom_pose_curr = pose_curr;
        Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

        double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
        translationAccumulated += delta_translation;
        rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

        if( translationAccumulated > keyframeMeterGap_ || rotaionAccumulated > keyframeRadGap_ ) {
            isNowKeyFrame = true;
            translationAccumulated = 0.0; // reset 
            rotaionAccumulated = 0.0; // reset 
        } else {
            isNowKeyFrame = false;
        }

        if( ! isNowKeyFrame ) 
            continue; 

        //
        // Save data and Add consecutive node 
        //
        pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
        downSizeFilterScancontext.setInputCloud(thisKeyFrame);
        downSizeFilterScancontext.filter(*thisKeyFrameDS);

        keyframeLaserClouds.push_back(thisKeyFrameDS);
        keyframePoses.push_back(pose_curr);
        keyframePosesUpdated.push_back(pose_curr); // init
        keyframeTimes.push_back(timeLaserOdometry);

        scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

        laserCloudMapPGORedraw = true;

        const int prev_node_idx = keyframePoses.size() - 2; 
        const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
        if( ! gtSAMgraphMade /* prior node */) {
            const int init_node_idx = 0; 
            gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
            // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

            // prior factor 
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
            initialEstimate.insert(init_node_idx, poseOrigin);
            // runISAM2opt();          

            gtSAMgraphMade = true; 

            logFile_ << "posegraph prior node " << init_node_idx << " added" << std::endl;
        } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
            gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
            gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

            // odom factor
            gtsam::Pose3 relPose = poseFrom.between(poseTo);
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));

            initialEstimate.insert(curr_node_idx, poseTo);                
            writeEdge({prev_node_idx, curr_node_idx}, relPose, edges_str); // giseop
            // runISAM2opt();

            // if(curr_node_idx % 100 == 0)
            logFile_ << "posegraph odom node " << curr_node_idx << " added." << std::endl;
        }
        // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

        // TODO: Add a flag to config.
        // // save utility 
        // std::string curr_node_idx_str = padZeros(curr_node_idx);
        // pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan 

        // const auto& curr_scd = scManager.getConstRefRecentSCD();
        // saveSCD(pgSCDsDirectory + curr_node_idx_str + ".scd", curr_scd);

        // pgTimeSaveStream << timeLaser << std::endl; // path 
    }

    // ps. 
    // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
    // pub path and point cloud in another thread

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
}

void PoseGraphOptimization::performSCLoopClosure(void) {
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        logFile_ << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << std::endl;

        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
    }
}

void PoseGraphOptimization::process_lcd(void) {
    performSCLoopClosure();
}

void PoseGraphOptimization::process_icp(void) {
    while ( !scLoopICPBuf.empty() ) {
        if( scLoopICPBuf.size() > 30 ) { // TODO: Move to config
            logFile_ << "Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)" << std::endl;
        }

        std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
        scLoopICPBuf.pop();

        const int prev_node_idx = loop_idx_pair.first;
        const int curr_node_idx = loop_idx_pair.second;
        auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
        if(relative_pose_optional) {
            gtsam::Pose3 relative_pose = relative_pose_optional.value();
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
            writeEdge({prev_node_idx, curr_node_idx}, relative_pose, edges_str); // giseop
            // runISAM2opt();
        } 
    }

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
}

void PoseGraphOptimization::process_isam(void) {
    if( gtSAMgraphMade ) {
        runISAM2opt();
        logFile_ << "running isam2 optimization ..." << std::endl;

        saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat_); // pose
        saveOdometryVerticesKITTIformat(odomKITTIformat_); // pose
        // saveGTSAMgraphG2oFormat(isamCurrentEstimate);
    }
}

// template <typename PointType>
void PoseGraphOptimization::eigenToPCL(const std::vector<Eigen::Vector4d> &ptCloud) {
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->width = ptCloud.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < ptCloud.size(); ++i) {
        PointType pt;
        pt.x = static_cast<float>(ptCloud[i][0]);
        pt.y = static_cast<float>(ptCloud[i][1]);
        pt.z = static_cast<float>(ptCloud[i][2]);
        cloud->points[i] = pt;
    }
    laserCloudFullResHandler(cloud); // update the full res cloud
    // return cloud;
}
