#include "rosWarper.hpp"

RIO::RIO(ros::NodeHandle &nh) {
  initROS(nh);
  initRIO();
}

RIO::~RIO() {}

void RIO::initROS(ros::NodeHandle &nh) {
  nodeHandle = nh;
  getParam();
}

void RIO::getParam() {
  ros::param::get("~useWeightedResiduals", useWeightedResiduals);

  ros::param::get("~useDopplerResidual", useDopplerResidual);
  ros::param::get("~usePoint2PointResidual", usePoint2PointResidual);

  ros::param::get("~SigmaRange", SigmaRange);
  ros::param::get("~SigmaAzimuth", SigmaAzimuth);
  ros::param::get("~SigmaElevation", SigmaElevation);
  ros::param::get("~SigmaDoppler", SigmaDoppler);
  ros::param::get("~numSigma", numSigma);
  ros::param::get("~maxVelInfoGain", maxVelInfoGain);
  ros::param::get("~maxPointInfoGain", maxPointInfoGain);
  ros::param::get("~dopplerResidualWeight", dopplerResidualWeight);
  ros::param::get("~pointResidualWeight", pointResidualWeight);

  ros::param::get("~radarSubTopic", radarSubTopic);
  ros::param::get("~imuSubTopic", imuSubTopic);
  ros::param::get("~nav_gtSubTopic", nav_gtSubTopic);
  ros::param::get("~geometry_gtSubTopic", geometry_gtSubTopic);
  ros::param::get("~posePubTopic", posePubTopic);
  ros::param::get("~pathPubTopic", pathPubTopic);
  ros::param::get("~gtPubTopic", gtPubTopic);
  ros::param::get("~framePubTopic", framePubTopic);
  ros::param::get("~trackingPubTopic", trackingPubTopic);
  ros::param::get("~subMapPubTopic", subMapPubTopic);

  int radarTypeInt = 0;
  ros::param::get("~radarType", radarTypeInt);
  radarType = static_cast<RadarType>(radarTypeInt);

  // Print parameters
  std::cout << "useWeightedResiduals: " << useWeightedResiduals << std::endl
            << "useRCSFilter: " << useRCSFilter << std::endl
            << "observationThreshold: " << observationThreshold << std::endl
            << "useDopplerResidual: " << useDopplerResidual << std::endl
            << "usePoint2PointResidual: " << usePoint2PointResidual << std::endl
            << "SigmaRange: " << SigmaRange << std::endl
            << "SigmaAzimuth: " << SigmaAzimuth << std::endl
            << "SigmaElevation: " << SigmaElevation << std::endl
            << "SigmaDoppler: " << SigmaDoppler << std::endl
            << "numSigma: " << numSigma << std::endl
            << "maxVelInfoGain: " << maxVelInfoGain << std::endl
            << "maxPointInfoGain: " << maxPointInfoGain << std::endl
            << "dopplerResidualWeight: " << dopplerResidualWeight << std::endl
            << "pointResidualWeight: " << pointResidualWeight << std::endl
            << "radarSubTopic: " << radarSubTopic << std::endl
            << "imuSubTopic: " << imuSubTopic << std::endl
            << "nav_gtSubTopic: " << nav_gtSubTopic << std::endl
            << "geometry_gtSubTopic: " << geometry_gtSubTopic << std::endl
            << "posePubTopic: " << posePubTopic << std::endl
            << "pathPubTopic: " << pathPubTopic << std::endl
            << "gtPubTopic: " << gtPubTopic << std::endl
            << "framePubTopic: " << framePubTopic << std::endl
            << "trackingPubTopic: " << trackingPubTopic << std::endl
            << "subMapPubTopic: " << subMapPubTopic << std::endl
            << "radarType: " << radarType << std::endl;
}

std::vector<Frame::RadarData> RIO::decodeRadarMsg_ARS548(
    const sensor_msgs::PointCloud2 &msg) {
  std::vector<Frame::RadarData> result;
  int pointBytes = static_cast<int>(msg.point_step);
  int offsetAzimuth = 0;
  int offsetAzimuthSTD = 0;
  int offsetElevation = 0;
  int offsetElevationSTD = 0;
  int offsetRange = 0;
  int offsetRangeSTD = 0;
  int offsetVelocity = 0;
  int offsetVelocitySTD = 0;
  int offsetRCS = 0;

  const auto &fields = msg.fields;
  for (auto i = 0; i < fields.size(); i++) {
    if (fields[i].name == "azimuth")
      offsetAzimuth = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "azimuthSTD")
      offsetAzimuthSTD = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "elevation")
      offsetElevation = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "elevationSTD")
      offsetElevationSTD = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "range")
      offsetRange = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "rangeSTD")
      offsetRangeSTD = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "velocity")
      offsetVelocity = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "velocitySTD")
      offsetVelocitySTD = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "rcs")
      offsetRCS = static_cast<int>(fields[i].offset);
  }
  pcl::PointCloud<pcl::PointXYZI> pointCloud;

  for (auto i = 0; i < msg.row_step; i++) {
    float azimuth = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetAzimuth));
    float azimuthSTD = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetAzimuthSTD));
    float elevation = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetElevation));
    float elevationSTD = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetElevationSTD));
    float range = *reinterpret_cast<const float *>(
        msg.data.data() + static_cast<ptrdiff_t>(pointBytes * i + offsetRange));
    float rangeSTD = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetRangeSTD));
    float velocity = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetVelocity));
    float velocitySTD = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetVelocitySTD));
    int8_t rcs = *reinterpret_cast<const int8_t *>(
        msg.data.data() + static_cast<ptrdiff_t>(pointBytes * i + offsetRCS));

    Frame::RadarData currentData{};
    currentData.azimuth = azimuth;
    currentData.elevation = elevation;
    currentData.range = range;
    currentData.doppler = velocity;
    currentData.rcs = rcs;
    Frame::RadarData::anglesToXYZ(currentData);
    Frame::RadarData::rcsToIntensity(currentData);
    result.emplace_back(currentData);

    pcl::PointXYZI point;
    point.x = range * cos(azimuth) * cos(elevation);
    point.y = range * sin(azimuth) * cos(elevation);
    point.z = range * sin(elevation);
    point.intensity = rcs;

    pointCloud.emplace_back(point);
  }

  // pointCloud to world frame
  // pcl::PointCloud<pcl::PointXYZI> pointCloudWorld;
  // pcl::transformPointCloud(pointCloud, pointCloudWorld, gtTransform);
  // std::string path = "/ws/src/dataset/exp/square_fast/";
  // std::string filename = path + std::to_string(msg.header.stamp.toSec()) +
  // ".pcd"; pcl::io::savePCDFileASCII(filename, pointCloudWorld);

  sensor_msgs::PointCloud2 pubMsg;
  pcl::toROSMsg(pointCloud, pubMsg);
  pubMsg.header.frame_id = "world";
  framePub.publish(pubMsg);
  return result;
}

std::vector<Frame::RadarData> RIO::decodeRadarMsg_ColoRadar(
    const sensor_msgs::PointCloud2 &msg) {
  std::vector<Frame::RadarData> result;
  int pointBytes = static_cast<int>(msg.point_step);
  int offsetX = 0;
  int offsetY = 0;
  int offsetZ = 0;
  int offsetIntensity = 0;
  int offsetRange = 0;
  int offsetDoppler = 0;
  // auto receivedTime = msg.header.stamp;

  const auto &fields = msg.fields;
  for (auto i = 0; i < fields.size(); i++) {
    if (fields[i].name == "x")
      offsetX = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "y")
      offsetY = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "z")
      offsetZ = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "intensity")
      offsetIntensity = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "range")
      offsetRange = static_cast<int>(fields[i].offset);
    else if (fields[i].name == "doppler")
      offsetDoppler = static_cast<int>(fields[i].offset);
  }
  pcl::PointCloud<pcl::PointXYZI> pointCloud;

  for (auto i = 0; i < msg.width; i++) {
    float xValue = *reinterpret_cast<const float *>(
        msg.data.data() + static_cast<ptrdiff_t>(pointBytes * i + offsetX));
    float yValue = *reinterpret_cast<const float *>(
        msg.data.data() + static_cast<ptrdiff_t>(pointBytes * i + offsetY));
    float zValue = *reinterpret_cast<const float *>(
        msg.data.data() + static_cast<ptrdiff_t>(pointBytes * i + offsetZ));
    float range = *reinterpret_cast<const float *>(
        msg.data.data() + static_cast<ptrdiff_t>(pointBytes * i + offsetRange));
    float velocity = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetDoppler));
    float intensity = *reinterpret_cast<const float *>(
        msg.data.data() +
        static_cast<ptrdiff_t>(pointBytes * i + offsetIntensity));

    Frame::RadarData currentData{};
    currentData.x = xValue;
    currentData.y = yValue;
    currentData.z = zValue;
    currentData.range = range;
    currentData.doppler = velocity;
    currentData.rcs = intensity;
    // Frame::RadarData::xyzToAngles(currentData);
    // Frame::RadarData::intensityToRCS(currentData);
    result.emplace_back(currentData);

    pcl::PointXYZI point;
    point.x = xValue;
    point.y = yValue;
    point.z = zValue;
    point.intensity = intensity;

    pointCloud.emplace_back(point);
  }

  sensor_msgs::PointCloud2 pubMsg;
  pcl::toROSMsg(pointCloud, pubMsg);
  pubMsg.header.frame_id = "world";
  framePub.publish(pubMsg);
  return result;
}

void RIO::factorGraphInit(std::vector<Frame::RadarData> &frameRadarData,
                          const ros::Time &timeStamp) {
  for (auto pts : frameRadarData) {
    Eigen::Vector3d p{pts.x, pts.y, pts.z};
    p = radarExParam.rot * p + radarExParam.vec;
    pcl::PointXYZI point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    point.intensity = pts.rcs;
    worldMap->emplace_back(point);
  }

  states.basicState[0].positionParams[0] = predState.vec.x();
  states.basicState[0].positionParams[1] = predState.vec.y();
  states.basicState[0].positionParams[2] = predState.vec.z();

  states.basicState[0].rotationParams[0] = predState.rot.x();
  states.basicState[0].rotationParams[1] = predState.rot.y();
  states.basicState[0].rotationParams[2] = predState.rot.z();
  states.basicState[0].rotationParams[3] = predState.rot.w();

  states.imuState[0].velocityParams[0] = predState.vel.x();
  states.imuState[0].velocityParams[1] = predState.vel.y();
  states.imuState[0].velocityParams[2] = predState.vel.z();

  states.imuState[0].biasAccParams[0] = predState.accBias.x();
  states.imuState[0].biasAccParams[1] = predState.accBias.y();
  states.imuState[0].biasAccParams[2] = predState.accBias.z();

  states.imuState[0].biasGyroParams[0] = predState.gyroBias.x();
  states.imuState[0].biasGyroParams[1] = predState.gyroBias.y();
  states.imuState[0].biasGyroParams[2] = predState.gyroBias.z();
  states.basicStateNum = 1;
  states.imuStateNum = 1;

  radarData.data.emplace_back(
      scan2scanTracker.trackPoints(frameRadarData, timeStamp));
  radarData.data.back().gyroData = imuData.data.back().gyroData;
  radarFeatureFactor.clear();
  radarFeatureFactor.pushBack(radarData.data.back());
}

Residuals::Preintegration RIO::imuPreintegration(ros::Time start,
                                                 ros::Time end) {
  std::vector<Frame::IMUFrame> dataBuffer =
      imuData.getDataWithinInterval(start, end);
  if (dataBuffer.empty())
    return Residuals::Preintegration{Eigen::Vector3d{0, 0, 0},
                                     Eigen::Vector3d{0, 0, 0},
                                     predState.accBias,
                                     predState.gyroBias,
                                     0.1,
                                     0.01,
                                     0.001,
                                     0.0001};
  Residuals::Preintegration result{dataBuffer.front().accData,
                                   dataBuffer.front().gyroData,
                                   predState.accBias,
                                   predState.gyroBias,
                                   0.1,
                                   0.01,
                                   0.001,
                                   0.0001};
  for (auto data = dataBuffer.begin(); data != dataBuffer.end(); data++) {
    if (data == dataBuffer.begin()) {
      result.propagate((data->receivedTime - start).toSec(), data->accData,
                       data->gyroData);
    } else if (data == (dataBuffer.end() - 1)) {
      result.propagate((data->receivedTime - (data - 1)->receivedTime).toSec(),
                       data->accData, data->gyroData);
      result.propagate((end - data->receivedTime).toSec(), data->accData,
                       data->gyroData);
    } else {
      result.propagate((data->receivedTime - (data - 1)->receivedTime).toSec(),
                       data->accData, data->gyroData);
    }
  }
  return result;
}

void RIO::constructFactor(std::vector<Frame::RadarData> &frameRadarData,
                          const ros::Time &timeStamp) {
  Residuals::Preintegration preintegration =
      imuPreintegration(radarData.data.back().receivedTime, timeStamp);
  // initial guess
  curState = predState;

  // predict
  predState.vec = curState.rot * preintegration.getDeltaP() +
                  curState.vel * preintegration.getTotalTime() + curState.vec -
                  0.5 * gravity * preintegration.getTotalTime() *
                      preintegration.getTotalTime();
  predState.rot = curState.rot * preintegration.getDeltaQ();
  predState.vel = curState.rot * preintegration.getDeltaV() + curState.vel -
                  gravity * preintegration.getTotalTime();

  Eigen::Vector3d unbiasedAngularVel =
      imuData.data.back().gyroData - predState.gyroBias;
  Eigen::Vector3d tangentVel =
      MathUtility::skewSymmetric(unbiasedAngularVel) * -radarExParam.vec;
  Eigen::Vector3d radialVel = predState.rot.inverse() * predState.vel;
  Eigen::Vector3d velInRadar =
      radarExParam.rot.inverse() * (radialVel + tangentVel);

  Eigen::Quaterniond curRotRadar = curState.rot * radarExParam.rot;
  Eigen::Vector3d curVecRadar = curState.rot * radarExParam.vec + curState.vec;
  Eigen::Quaterniond predRotRadar = predState.rot * radarExParam.rot;
  Eigen::Vector3d predVecRadar =
      predState.rot * radarExParam.vec + predState.vec;

  Frame::RadarFrame frame;
  scan2scanTracker.setPrediction(curRotRadar, curVecRadar, predRotRadar,
                                 predVecRadar, -velInRadar);
  frame = scan2scanTracker.trackPoints(frameRadarData, timeStamp);

  radarData.data.emplace_back(frame);
  radarData.data.back().gyroData = imuData.data.back().gyroData;
  radarFeatureFactor.pushBack(radarData.data.back());
  imuFeatureFactor.pushBack(preintegration, states.imuStateNum - 1,
                            states.imuStateNum);
}

void RIO::optimizer() {
  ceres::Problem problem;
  ceres::Solver::Options option;
  ceres::Solver::Summary summary;

  option.linear_solver_type = ceres::DENSE_SCHUR;
  option.max_solver_time_in_seconds = 0.2;
  option.max_num_iterations = 20;
  option.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

#ifdef DEBUG
  option.minimizer_progress_to_stdout = true;
  option.logging_type = ceres::PER_MINIMIZER_ITERATION;
#endif

  initStates();
  constructProblem(problem);
  ceres::Solve(option, &problem, &summary);
  // std::cout << summary.FullReport() << std::endl;
  recoverState(ceres::CONVERGENCE);
}

void RIO::initStates() {
  states.basicState[states.basicStateNum].positionParams[0] = predState.vec.x();
  states.basicState[states.basicStateNum].positionParams[1] = predState.vec.y();
  states.basicState[states.basicStateNum].positionParams[2] = predState.vec.z();

  states.basicState[states.basicStateNum].rotationParams[0] = predState.rot.x();
  states.basicState[states.basicStateNum].rotationParams[1] = predState.rot.y();
  states.basicState[states.basicStateNum].rotationParams[2] = predState.rot.z();
  states.basicState[states.basicStateNum].rotationParams[3] = predState.rot.w();

  states.imuState[states.imuStateNum].velocityParams[0] = predState.vel.x();
  states.imuState[states.imuStateNum].velocityParams[1] = predState.vel.y();
  states.imuState[states.imuStateNum].velocityParams[2] = predState.vel.z();

  states.imuState[states.imuStateNum].biasAccParams[0] = predState.accBias.x();
  states.imuState[states.imuStateNum].biasAccParams[1] = predState.accBias.y();
  states.imuState[states.imuStateNum].biasAccParams[2] = predState.accBias.z();

  states.imuState[states.imuStateNum].biasGyroParams[0] =
      predState.gyroBias.x();
  states.imuState[states.imuStateNum].biasGyroParams[1] =
      predState.gyroBias.y();
  states.imuState[states.imuStateNum].biasGyroParams[2] =
      predState.gyroBias.z();

  states.basicStateNum++;
  states.imuStateNum++;

  states.exRadarToImu.positionParams[0] = radarExParam.vec.x();
  states.exRadarToImu.positionParams[1] = radarExParam.vec.y();
  states.exRadarToImu.positionParams[2] = radarExParam.vec.z();

  states.exRadarToImu.rotationParams[0] = radarExParam.rot.x();
  states.exRadarToImu.rotationParams[1] = radarExParam.rot.y();
  states.exRadarToImu.rotationParams[2] = radarExParam.rot.z();
  states.exRadarToImu.rotationParams[3] = radarExParam.rot.w();
}

void RIO::recoverState(ceres::TerminationType type) {
  if (type != ceres::CONVERGENCE) {
    ROS_ERROR("Optimization failed");
    return;
  }
  predState.vec.x() =
      states.basicState[states.basicStateNum - 1].positionParams[0];
  predState.vec.y() =
      states.basicState[states.basicStateNum - 1].positionParams[1];
  predState.vec.z() =
      states.basicState[states.basicStateNum - 1].positionParams[2];

  predState.rot.x() =
      states.basicState[states.basicStateNum - 1].rotationParams[0];
  predState.rot.y() =
      states.basicState[states.basicStateNum - 1].rotationParams[1];
  predState.rot.z() =
      states.basicState[states.basicStateNum - 1].rotationParams[2];
  predState.rot.w() =
      states.basicState[states.basicStateNum - 1].rotationParams[3];

  predState.vel.x() =
      states.imuState[states.basicStateNum - 1].velocityParams[0];
  predState.vel.y() =
      states.imuState[states.basicStateNum - 1].velocityParams[1];
  predState.vel.z() =
      states.imuState[states.basicStateNum - 1].velocityParams[2];

  predState.accBias.x() =
      states.imuState[states.basicStateNum - 1].biasAccParams[0];
  predState.accBias.y() =
      states.imuState[states.basicStateNum - 1].biasAccParams[1];
  predState.accBias.z() =
      states.imuState[states.basicStateNum - 1].biasAccParams[2];

  predState.gyroBias.x() =
      states.imuState[states.basicStateNum - 1].biasGyroParams[0];
  predState.gyroBias.y() =
      states.imuState[states.basicStateNum - 1].biasGyroParams[1];
  predState.gyroBias.z() =
      states.imuState[states.basicStateNum - 1].biasGyroParams[2];

  radarExParam.vec.x() = states.exRadarToImu.positionParams[0];
  radarExParam.vec.y() = states.exRadarToImu.positionParams[1];
  radarExParam.vec.z() = states.exRadarToImu.positionParams[2];

  radarExParam.rot.x() = states.exRadarToImu.rotationParams[0];
  radarExParam.rot.y() = states.exRadarToImu.rotationParams[1];
  radarExParam.rot.z() = states.exRadarToImu.rotationParams[2];
  radarExParam.rot.w() = states.exRadarToImu.rotationParams[3];

  if (states.basicStateNum > State::MAX_WINDOWS_SIZE) {
    for (int i = 0; i < states.basicStateNum - 1; i++) {
      states.basicState[i].positionParams[0] =
          states.basicState[i + 1].positionParams[0];
      states.basicState[i].positionParams[1] =
          states.basicState[i + 1].positionParams[1];
      states.basicState[i].positionParams[2] =
          states.basicState[i + 1].positionParams[2];
      states.basicState[i].rotationParams[0] =
          states.basicState[i + 1].rotationParams[0];
      states.basicState[i].rotationParams[1] =
          states.basicState[i + 1].rotationParams[1];
      states.basicState[i].rotationParams[2] =
          states.basicState[i + 1].rotationParams[2];
      states.basicState[i].rotationParams[3] =
          states.basicState[i + 1].rotationParams[3];

      states.imuState[i].velocityParams[0] =
          states.imuState[i + 1].velocityParams[0];
      states.imuState[i].velocityParams[1] =
          states.imuState[i + 1].velocityParams[1];
      states.imuState[i].velocityParams[2] =
          states.imuState[i + 1].velocityParams[2];
      states.imuState[i].biasAccParams[0] =
          states.imuState[i + 1].biasAccParams[0];
      states.imuState[i].biasAccParams[1] =
          states.imuState[i + 1].biasAccParams[1];
      states.imuState[i].biasAccParams[2] =
          states.imuState[i + 1].biasAccParams[2];
      states.imuState[i].biasGyroParams[0] =
          states.imuState[i + 1].biasGyroParams[0];
      states.imuState[i].biasGyroParams[1] =
          states.imuState[i + 1].biasGyroParams[1];
      states.imuState[i].biasGyroParams[2] =
          states.imuState[i + 1].biasGyroParams[2];
    }
    states.basicStateNum--;
    states.imuStateNum--;
    radarFeatureFactor.popFront();
    imuFeatureFactor.popFront();
  }
}

void RIO::constructProblem(ceres::Problem &problem) {
  addOptimizationVariables(problem);
  constructImuResiduals(problem);
  if (useDopplerResidual) {
    constructDopplerResiduals(problem);
  }
  if (usePoint2PointResidual) {
    constructPoint2PointResiduals(problem);
  }
}

void RIO::addOptimizationVariables(ceres::Problem &problem) {
  // extrinsic
  problem.AddParameterBlock(states.exRadarToImu.positionParams, 3);
  problem.AddParameterBlock(states.exRadarToImu.rotationParams, 4);
  problem.SetManifold(states.exRadarToImu.rotationParams,
                      // NOLINTNEXTLINE : ceres will take ownership of this
                      new Manifold::EigenQuaternionManifold());
  problem.SetParameterBlockConstant(states.exRadarToImu.positionParams);
  problem.SetParameterBlockConstant(states.exRadarToImu.rotationParams);

  // state
  for (int offset = 0; offset < states.basicStateNum; offset++) {
    problem.AddParameterBlock(states.basicState[offset].positionParams, 3);
    problem.AddParameterBlock(states.basicState[offset].rotationParams, 4);
    problem.SetManifold(states.basicState[offset].rotationParams,
                        new Manifold::EigenQuaternionManifold());
    problem.AddParameterBlock(states.imuState[offset].velocityParams, 3);
    problem.AddParameterBlock(states.imuState[offset].biasAccParams, 3);
    problem.AddParameterBlock(states.imuState[offset].biasGyroParams, 3);
  }

  problem.SetParameterBlockConstant(states.basicState[0].positionParams);
  problem.SetParameterBlockConstant(states.basicState[0].rotationParams);
  problem.SetParameterBlockConstant(states.imuState[0].velocityParams);
  problem.SetParameterBlockConstant(states.imuState[0].biasAccParams);
  problem.SetParameterBlockConstant(states.imuState[0].biasGyroParams);
}

void RIO::constructImuResiduals(ceres::Problem &problem) {
  for (int offset = 0; offset < states.basicStateNum - 1; offset++) {
    auto preintegrationTemp = imuFeatureFactor.imuPreintegration[offset];
    auto *imuCost = new Residuals::IMUResidual(preintegrationTemp, gravity);
    problem.AddResidualBlock(imuCost, nullptr,
                             states.basicState[offset].positionParams,
                             states.basicState[offset].rotationParams,
                             states.imuState[offset].velocityParams,
                             states.imuState[offset].biasAccParams,
                             states.imuState[offset].biasGyroParams,
                             states.basicState[offset + 1].positionParams,
                             states.basicState[offset + 1].rotationParams,
                             states.imuState[offset + 1].velocityParams,
                             states.imuState[offset + 1].biasAccParams,
                             states.imuState[offset + 1].biasGyroParams);
  }
}

void RIO::constructDopplerResiduals(ceres::Problem &problem) {
  auto beginFrame = radarData.data.end() - 1 - states.basicStateNum;
  int staticPointSize = 0;
  for (int offset = 0; offset < states.basicStateNum; offset++)
    staticPointSize += (beginFrame + offset)->staticPoint.size();

  double *staticPointState = new double[3 * staticPointSize];
  int staticPointCount = 0;
  for (int offset = 0; offset < states.basicStateNum; offset++) {
    for (auto point : (beginFrame + offset)->staticPoint) {
      Eigen::Vector3d angularVel = (beginFrame + offset)->gyroData;
      Eigen::Vector3d targetPos{point.x, point.y, point.z};
      staticPointState[3 * staticPointCount] = targetPos.x();
      staticPointState[3 * staticPointCount + 1] = targetPos.y();
      staticPointState[3 * staticPointCount + 2] = targetPos.z();
      problem.AddParameterBlock(staticPointState + 3 * staticPointCount, 3);
      problem.SetParameterBlockConstant(staticPointState +
                                        3 * staticPointCount);

      Residuals::RadarImuVelocityResidual *costFn;
      if (useWeightedResiduals) {
        costFn = new Residuals::RadarImuVelocityResidual(
            angularVel, -point.doppler, point.azimuth, point.elevation,
            SigmaAzimuth, SigmaElevation, maxVelInfoGain);
      } else {
        costFn = new Residuals::RadarImuVelocityResidual(
            angularVel, -point.doppler, dopplerResidualWeight);
      }
      problem.AddResidualBlock(costFn, nullptr,
                               states.basicState[offset].rotationParams,
                               states.imuState[offset].velocityParams,
                               states.imuState[offset].biasGyroParams,
                               states.exRadarToImu.positionParams,
                               states.exRadarToImu.rotationParams,
                               staticPointState + 3 * staticPointCount);
      staticPointCount++;
    }
  }
}

void RIO::constructPoint2PointResiduals(ceres::Problem &problem) {
  states.pointNum = 0;
  for (auto &point : radarFeatureFactor.pointRelation) {
    if (point.frameId.size() < observationThreshold) continue;
    std::vector<int> stateIndex(point.frameId.size());
    for (int i = 0; i < point.frameId.size(); i++) {
      for (int j = 0; j < radarFeatureFactor.frameRelation.size(); j++)
        if (radarFeatureFactor.frameRelation[j].radarFrameId ==
            point.frameId[i])
          stateIndex[i] = j;
    }
    if (!point.init) {
      point.init = true;
      Eigen::Vector3d initGuess{point.measurement[0].x, point.measurement[0].y,
                                point.measurement[0].z};
      Eigen::Vector3d translation{states.basicState[0].positionParams[0],
                                  states.basicState[0].positionParams[1],
                                  states.basicState[0].positionParams[2]};
      Eigen::Quaterniond rotation{states.basicState[0].rotationParams[3],
                                  states.basicState[0].rotationParams[0],
                                  states.basicState[0].rotationParams[1],
                                  states.basicState[0].rotationParams[2]};
      initGuess = rotation * (radarExParam.rot * initGuess + radarExParam.vec) +
                  translation;
      point.pointState = initGuess;
    }

    states.pointState[states.pointNum].positionParams[0] = point.pointState.x();
    states.pointState[states.pointNum].positionParams[1] = point.pointState.y();
    states.pointState[states.pointNum].positionParams[2] = point.pointState.z();
    problem.AddParameterBlock(states.pointState[states.pointNum].positionParams,
                              3);
    for (int i = 0; i < point.frameId.size(); i++) {
      auto data = point.measurement[i];
      int observationCount = point.frameId.size();
      int pointNum = radarFeatureFactor.pointRelation.size();
      auto state = stateIndex[i];
      Eigen::Vector3d measurement{data.x, data.y, data.z};
      Residuals::RadarPointResidual *costFn;
      if (useWeightedResiduals) {
        costFn = new Residuals::RadarPointResidual(
            measurement, data.range, data.azimuth, data.elevation, SigmaRange,
            SigmaAzimuth, SigmaElevation, maxPointInfoGain, observationCount);
      } else {
        costFn =
            new Residuals::RadarPointResidual(measurement, pointResidualWeight);
      }
      problem.AddResidualBlock(
          costFn, nullptr, states.basicState[state].positionParams,
          states.basicState[state].rotationParams,
          states.exRadarToImu.positionParams,
          states.exRadarToImu.rotationParams,
          states.pointState[states.pointNum].positionParams);
    }
    states.pointNum++;
  }
}

void RIO::publish(const ros::Time &timeStamp) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr featurePoints(
      new pcl::PointCloud<pcl::PointXYZ>);

  int pointsNum = states.pointNum;
  if (pointsNum > 0) {
    for (auto &point : radarFeatureFactor.pointRelation) {
      if (point.frameId.size() < observationThreshold) continue;

      point.pointState.x() =
          states.pointState[pointsNum - states.pointNum].positionParams[0];
      point.pointState.y() =
          states.pointState[pointsNum - states.pointNum].positionParams[1];
      point.pointState.z() =
          states.pointState[pointsNum - states.pointNum].positionParams[2];

      pcl::PointXYZI mapPt;
      mapPt.x = point.pointState.x();
      mapPt.y = point.pointState.y();
      mapPt.z = point.pointState.z();
      mapPt.intensity = point.measurement.front().rcs;
      worldMap->emplace_back(mapPt);

      pcl::PointXYZ pt;
      pt.x = point.pointState.x();
      pt.y = point.pointState.y();
      pt.z = point.pointState.z();
      featurePoints->emplace_back(pt);

      states.pointNum--;
    }
  }

  sensor_msgs::PointCloud2 featureMsg;
  pcl::toROSMsg(*featurePoints, featureMsg);
  featureMsg.header.frame_id = "world";
  trackingPub.publish(featureMsg);

  geometry_msgs::PoseStamped pubMsg;
  pubMsg.header.frame_id = "world";
  pubMsg.header.stamp = timeStamp;
  pubMsg.pose.orientation.w = predState.rot.w();
  pubMsg.pose.orientation.x = predState.rot.x();
  pubMsg.pose.orientation.y = predState.rot.y();
  pubMsg.pose.orientation.z = predState.rot.z();
  pubMsg.pose.position.x = predState.vec.x();
  pubMsg.pose.position.y = predState.vec.y();
  pubMsg.pose.position.z = predState.vec.z();
  posePub.publish(pubMsg);

  // visualize path
  path.poses.emplace_back(pubMsg);
  path.header.frame_id = "world";
  pathPub.publish(path);

  // visualize map
  sensor_msgs::PointCloud2 mapMsg;
  pcl::toROSMsg(*worldMap, mapMsg);
  mapMsg.header.frame_id = "world";
  subMapPub.publish(mapMsg);
}

void RIO::radarCallback(const sensor_msgs::PointCloud2 &msg) {
  if (imuData.size() <= 0) {
    ROS_ERROR("No imu data");
    return;
  }

  // Preprocess radar data
  std::vector<Frame::RadarData> frameRadarData;
  if (radarType == ARS548)
    frameRadarData = decodeRadarMsg_ARS548(msg);
  else if (radarType == ColoRadar)
    frameRadarData = decodeRadarMsg_ColoRadar(msg);
  else
    ROS_ERROR("Unknown radar type");

  radarPreprocessor.process(frameRadarData);

  // Initialize factor graph
  if ((radarData.size() <= 3 || !init)) {
    factorGraphInit(frameRadarData, msg.header.stamp);
    return;
  }

  // Construct factor
  constructFactor(frameRadarData, msg.header.stamp);

  // Optimize
  optimizer();

  // Publish
  publish(msg.header.stamp);
}

static int initCounter = 0;
void RIO::imuCallback(const sensor_msgs::Imu &msg) {
  auto frame = Frame::IMUFrame();
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
  if (radarType == ARS548) {
    acc = Eigen::Vector3d{msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z};
    gyro = Eigen::Vector3d{msg.angular_velocity.x, msg.angular_velocity.y,
                           msg.angular_velocity.z};
  } else if (radarType == ColoRadar) {
    acc = Eigen::Vector3d{msg.linear_acceleration.x, -msg.linear_acceleration.y,
                          -msg.linear_acceleration.z};
    gyro = Eigen::Vector3d{msg.angular_velocity.x, -msg.angular_velocity.y,
                           -msg.angular_velocity.z};
  } else {
    ROS_ERROR("Unknown radar type");
  }
  frame.receivedTime = msg.header.stamp;
  frame.accData = acc;
  frame.gyroData = gyro;
  imuData.push(frame);
  if (!init) {
    initCounter++;
    gravity += acc;
    if (initCounter >= 100) {
      gravity /= initCounter;
      curState.vec.setZero();
      curState.rot.setIdentity();
      curState.vel.setZero();
      curState.accBias.setZero();
      curState.gyroBias.setZero();

      predState.vec.setZero();
      predState.rot.setIdentity();
      predState.vel.setZero();
      predState.accBias.setZero();
      predState.gyroBias.setZero();

      if (radarType == ARS548) {
        radarExParam.vec.setZero();
        radarExParam.vec.z() = -0.05;
        radarExParam.rot.setIdentity();
      } else if (radarType == ColoRadar) {
        Eigen::Quaterniond quatImuToBase(0.00020953429822, 0.707105451466,
                                         0.707108048814, 0.000209535067884);
        Eigen::Vector3d vecImuToBase(0, 0, 0);
        Eigen::Vector3d vecRadarToBase{-0.145, 0.09, -0.025};
        Eigen::Quaterniond quatRadarToBase{0.707388269167, 0.0, 0.0,
                                           0.706825181105};
        radarExParam.vec = Eigen::Vector3d{-0.09, -0.145, 0.025};
        radarExParam.rot.setIdentity();
      } else {
        ROS_ERROR("Unknown radar type");
      }

      init = true;
      std::cout << "g:\n" << gravity << std::endl;
    }
  }
}

static Eigen::Quaterniond z90rot(Eigen::AngleAxisd(M_PI / 2,
                                                   Eigen::Vector3d::UnitZ()));
static Eigen::Vector3d firstVec;
static Eigen::Quaterniond firstRot;
static bool firstFrame = true;
void RIO::geometry_gtCallback(const geometry_msgs::PoseStamped &msg) {
  if (firstFrame) {
    firstVec = Eigen::Vector3d{msg.pose.position.x, msg.pose.position.y,
                               msg.pose.position.z};
    firstRot =
        Eigen::Quaterniond{msg.pose.orientation.w, msg.pose.orientation.x,
                           msg.pose.orientation.y, msg.pose.orientation.z};
    firstFrame = false;
  }
  auto vec = Eigen::Vector3d{msg.pose.position.x, msg.pose.position.y,
                             msg.pose.position.z};
  auto rot = Eigen::Quaterniond{msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z};
  vec = firstRot.inverse() * (vec - firstVec);
  rot = firstRot.inverse() * rot;

  geometry_msgs::PoseStamped pubMsg;
  pubMsg.header.frame_id = "world";
  pubMsg.header.stamp = msg.header.stamp;
  pubMsg.pose.orientation.w = rot.w();
  pubMsg.pose.orientation.x = rot.x();
  pubMsg.pose.orientation.y = rot.y();
  pubMsg.pose.orientation.z = rot.z();
  pubMsg.pose.position.x = vec.x();
  pubMsg.pose.position.y = vec.y();
  pubMsg.pose.position.z = vec.z();
  gtPub.publish(pubMsg);
}

void RIO::nav_gtCallback(const nav_msgs::Odometry &msg) {
  if (firstFrame) {
    firstVec =
        Eigen::Vector3d{msg.pose.pose.position.x, msg.pose.pose.position.y,
                        msg.pose.pose.position.z};
    firstRot = Eigen::Quaterniond{
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z};
    firstFrame = false;
  }
  auto vec = Eigen::Vector3d{msg.pose.pose.position.x, msg.pose.pose.position.y,
                             msg.pose.pose.position.z};
  auto rot = Eigen::Quaterniond{
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z};
  vec = firstRot.inverse() * (vec - firstVec);
  rot = firstRot.inverse() * rot * z90rot;

  geometry_msgs::PoseStamped pubMsg;
  pubMsg.header.frame_id = "world";
  pubMsg.header.stamp = msg.header.stamp;
  pubMsg.pose.orientation.w = rot.w();
  pubMsg.pose.orientation.x = rot.x();
  pubMsg.pose.orientation.y = rot.y();
  pubMsg.pose.orientation.z = rot.z();
  pubMsg.pose.position.x = vec.x();
  pubMsg.pose.position.y = vec.y();
  pubMsg.pose.position.z = vec.z();
  gtPub.publish(pubMsg);
}

void RIO::initRIO() {
  radarSub = nodeHandle.subscribe(radarSubTopic, SUB_QUEUE_SIZE,
                                  &RIO::radarCallback, this);
  imuSub = nodeHandle.subscribe(imuSubTopic, SUB_QUEUE_SIZE, &RIO::imuCallback,
                                this);
  nav_gtSub = nodeHandle.subscribe(nav_gtSubTopic, SUB_QUEUE_SIZE,
                                   &RIO::nav_gtCallback, this);
  geometry_gtSub = nodeHandle.subscribe(geometry_gtSubTopic, SUB_QUEUE_SIZE,
                                        &RIO::geometry_gtCallback, this);

  posePub = nodeHandle.advertise<geometry_msgs::PoseStamped>(posePubTopic,
                                                             PUB_QUEUE_SIZE);
  pathPub = nodeHandle.advertise<nav_msgs::Path>(pathPubTopic, PUB_QUEUE_SIZE);
  gtPub = nodeHandle.advertise<geometry_msgs::PoseStamped>(gtPubTopic,
                                                           PUB_QUEUE_SIZE);

  framePub = nodeHandle.advertise<sensor_msgs::PointCloud2>(framePubTopic,
                                                            PUB_QUEUE_SIZE);
  trackingPub = nodeHandle.advertise<sensor_msgs::PointCloud2>(trackingPubTopic,
                                                               PUB_QUEUE_SIZE);
  subMapPub = nodeHandle.advertise<sensor_msgs::PointCloud2>(subMapPubTopic,
                                                             PUB_QUEUE_SIZE);

  // radarPreprocessor
  if (radarType == ARS548) {
    radarPreprocessor.addFOVParams(Frontend::RadarPreprocessor::FOVParams{
        -14 * DEG_TO_RAD, 14 * DEG_TO_RAD, -50 * DEG_TO_RAD, +50 * DEG_TO_RAD,
        0.2, 30});

    radarPreprocessor.setVelParams(
        Frontend::RadarPreprocessor::VelParams{-100, 50});
    radarPreprocessor.setDistanceParams(1.5);
  } else if (radarType == ColoRadar) {
    radarPreprocessor.addFOVParams(Frontend::RadarPreprocessor::FOVParams{
        -14 * DEG_TO_RAD, 14 * DEG_TO_RAD, -28 * DEG_TO_RAD, 28 * DEG_TO_RAD,
        1.5, 15});
    radarPreprocessor.setVelParams(
        Frontend::RadarPreprocessor::VelParams{-30, 30});
    radarPreprocessor.setDistanceParams(0.5);
  }

  // radarTracker
  scan2scanTracker.setMatchingThreshold(1, 0.3);
  scan2scanTracker.setMatchingParameters(
      SigmaRange, SigmaAzimuth, SigmaElevation, numSigma, useRCSFilter);
  scan2scanTracker.setPredictedVelocityThreshold(0.3);
}