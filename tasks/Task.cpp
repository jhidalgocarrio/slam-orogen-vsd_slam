/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

using namespace vsd_slam;

/** Process model when accumulating delta poses **/
WMTKState processModel (const WMTKState &state,  const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity, const double delta_t)
{
    WMTKState s2; /** Propagated state */

    /** Update rotation rate **/
    s2.angvelo = angular_velocity;

    /** Apply Rotation **/
    ::vsd_slam::vec3 scaled_axis = state.angvelo * delta_t;
    s2.orient = state.orient * ::vsd_slam::SO3::exp(scaled_axis);

    /** Update the velocity (position rate) **/
    s2.velo = state.orient * linear_velocity;

    /** Apply Translation **/
    s2.pos = state.pos + (state.velo * delta_t);

    return s2;
};


Task::Task(std::string const& name)
    : TaskBase(name)
{
    /** Set pose and landmark symbol identifiers **/
    this->pose_key = 'x';
    this->landmark_key = 'l';
    this->pose_idx = 0;
    this->landmark_idx = 0;

    this->init_flag = false;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    /** Set pose and landmark symbol identifiers **/
    this->pose_key = 'x';
    this->landmark_key = 'l';
    this->pose_idx = 0;
    this->landmark_idx = 0;

    this->init_flag = false;
}

Task::~Task()
{
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{
    if (!this->init_flag)
    {
        /***************************
        * Get the Sensor frame
        * wrt the world frame
        ***************************/
        Eigen::Affine3d world_nav_tf; /** Transformer transformation **/

        /** Get the transformation Tworld_navigation (navigation is body_0) **/
        if (_navigation_frame.value().compare(_world_frame.value()) == 0)
        {
            world_nav_tf.setIdentity();
        }
        else if (!_navigation2world.get(ts, world_nav_tf, false))
        {
            RTT::log(RTT::Fatal)<<"[VSD_SLAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[VSD_SLAM POSE_SAMPLES] - Initializing Visual Stereo Back-End..."<<RTT::endlog();
        #endif

        /** Set initial pose out in body frame **/
        this->vs_pose_out.position = world_nav_tf.translation(); //!Initial position
        this->vs_pose_out.orientation = Eigen::Quaternion<double>(world_nav_tf.rotation());
        this->vs_pose_out.velocity.setZero(); //!Initial velocity
        this->vs_pose_out.angular_velocity.setZero(); //!Initial angular velocity

        /** Get the transformation Tbody_sensor **/
        Eigen::Affine3d body_sensor_tf; /** Transformer transformation **/
        if (_sensor_frame.value().compare(_body_frame.value()) == 0)
        {
            body_sensor_tf.setIdentity();
        }
        else if (!_sensor2body.get(ts, body_sensor_tf, false))
        {
            RTT::log(RTT::Fatal)<<"[VSD_SLAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        /** Store Tbody_sensor **/
        this->body_sensor_tf = body_sensor_tf;

        /** Change the initial pose out in sensor frame **/
        this->vs_pose_out.position = body_sensor_tf.inverse() * this->vs_pose_out.position; // p_sensor = Tsensor_body p_body
        this->vs_pose_out.orientation = this->vs_pose_out.orientation * Eigen::Quaternion <double>(body_sensor_tf.rotation()); //Tworld_sensor = Tworld_body * Tbody_sensor

        /***************************
        * BACK-END INITIALIZATION  *
        ***************************/
        Eigen::Affine3d vs_pose_out_tf = this->vs_pose_out.getTransform();
        this->initialization(vs_pose_out_tf);

        /** Increase pose index **/
        this->pose_idx++;

        /** Initialization succeeded **/
        this->init_flag = true;

        /** Set the initial delta_pose **/
        this->delta_pose = delta_pose_samples_sample;


        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[DONE]\n";
        #endif
    }

    /** Delta time between samples **/
    const double predict_delta_t = delta_pose_samples_sample.time.toSeconds() - this->delta_pose.time.toSeconds();

    Eigen::Affine3d body_sensor_tf; /** Transformer transformation **/

    /** Get the transformation Tbody_sensor **/
    if (_sensor_frame.value().compare(_body_frame.value()) == 0)
    {
        body_sensor_tf.setIdentity();
    }
    else if (!_sensor2body.get(ts, body_sensor_tf, false))
    {
        RTT::log(RTT::Fatal)<<"[VSD_SLAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
       return;
    }

    /** Calculate the delta pose Tb(k-1)_b(k) **/
    Eigen::Affine3d body_delta_pose_tf (delta_pose_samples_sample.orientation);
    body_delta_pose_tf.translation() = delta_pose_samples_sample.position;

    /** Calculate the delta pose in sensor(s) Ts(k-1)_s(k) **/
    /** Ts(k-1)_s(k) = Ts(k-1)_b(k-1) * Tb(k-1)_b(k) * Tb(k)_s(k) **/
    Eigen::Affine3d sensor_delta_pose_tf = this->body_sensor_tf.inverse() * body_delta_pose_tf * body_sensor_tf;

    /** Store the current delta_pose in sensor frame. Ts(k-1)_s(k) **/
    this->delta_pose.time = delta_pose_samples_sample.time;
    this->delta_pose.position = sensor_delta_pose_tf.translation();
    this->delta_pose.orientation =  Eigen::Quaternion <double>(sensor_delta_pose_tf.rotation());
    this->delta_pose.cov_position.setZero(); this->delta_pose.cov_orientation.setZero();
    this->delta_pose.cov_position.diagonal() = body_sensor_tf.rotation().inverse() * delta_pose_samples_sample.cov_position.diagonal();
    this->delta_pose.cov_orientation.diagonal() = body_sensor_tf.rotation().inverse() * delta_pose_samples_sample.cov_orientation.diagonal();

    /** Delta pose velocity in sensor frame vs(k-1) = Ts(k-1)_b(k-1) * vb(k-1) **/
    this->delta_pose.velocity = this->body_sensor_tf.inverse().rotation() * delta_pose_samples_sample.velocity;
    this->delta_pose.angular_velocity =  this->body_sensor_tf.inverse().rotation() * delta_pose_samples_sample.angular_velocity;
    this->delta_pose.cov_velocity = this->body_sensor_tf.inverse().rotation() * delta_pose_samples_sample.cov_velocity * this->body_sensor_tf.inverse().rotation().transpose();
    this->delta_pose.cov_angular_velocity = this->body_sensor_tf.inverse().rotation() * delta_pose_samples_sample.cov_angular_velocity * this->body_sensor_tf.inverse().rotation().transpose();

    /** Update the Tbody_sensor with the current one Tb(k)_s(k) **/
    this->body_sensor_tf = body_sensor_tf;

    /******************************************
    * Delta pose BetweenFactor in GTSAM
    * TO-DO: Remove it is not really needed
    * ****************************************/

    /** Delta noise model **/
    ::base::Matrix6d cov_delta_pose;
    cov_delta_pose << this->delta_pose.cov_position, Eigen::Matrix3d::Zero(),
                   Eigen::Matrix3d::Zero(), this->delta_pose.cov_orientation;

    /** Symbols **/
    gtsam::Symbol symbol1 = gtsam::Symbol(this->pose_key, this->pose_idx-1);
    gtsam::Symbol symbol2 = gtsam::Symbol(this->pose_key, this->pose_idx);

    /** Add the delta pose to the factor graph. TO-DO: probably not needed **/
    this->factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symbol1, symbol2,
                gtsam::Pose3(gtsam::Rot3(this->delta_pose.orientation), gtsam::Point3(this->delta_pose.position)),
                gtsam::noiseModel::Gaussian::Covariance(cov_delta_pose)));

    /******************************************
    * Delta pose integration in sensor frame *
    * ****************************************/

    /** Process Model Uncertainty **/
    UKF::cov cov_process; cov_process.setZero();
    MTK::subblock (cov_process, &WMTKState::velo, &WMTKState::velo) = this->delta_pose.cov_velocity;
    MTK::subblock (cov_process, &WMTKState::angvelo, &WMTKState::angvelo) = this->delta_pose.cov_angular_velocity;

    /** Predict the filter state **/
    this->filter->predict(boost::bind(processModel, _1 ,
                            static_cast<const Eigen::Vector3d>(this->delta_pose.velocity),
                            static_cast<const Eigen::Vector3d>(this->delta_pose.angular_velocity),
                            predict_delta_t),
                            cov_process);

    /***********************************************
    * Sensor Pose in GTSAM initial estimated values
    * **********************************************/
    gtsam::Pose3 current_pose(gtsam::Rot3(this->filter->mu().orient), gtsam::Point3(this->filter->mu().pos));
    this->sam_values.insert(symbol2, current_pose);

    /** Increase in one unit the pose index **/
    this->pose_idx++;

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[VSD_SLAM DELTA_POSE ] POSE ID: "<<symbol2<<RTT::endlog();
    #endif

    /** Output port the odometry pose **/
    this->odo_poseOutputport(this->delta_pose.time);
}

void Task::visual_features_samplesTransformerCallback(const base::Time &ts, const ::visual_stereo::ExteroFeatures &visual_features_samples_sample)
{

    if(!init_flag)
    {
        RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] FILTER STILL NOT INITIALIZED"<<RTT::endlog();
        return;
    }

    gtsam::Symbol symbol1 = gtsam::Symbol(this->landmark_key, this->landmark_idx);
    std::cout<<"[VSD_SLAM FEATURES ] LANDMARK ID: "<<symbol1<<"\n";
    //RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] LANDMARK ID: "<<symbol1<<RTT::endlog();
    this->landmark_idx++;

    /** Read Stereo measurement from the input port **/

    /** Set Generic Stereo Factor between measurements and the current camera pose **/

    /** Set the estimated landmark pose **/

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Read the camera calibration parameters **/
    this->camera_calib = _calib_parameters.value();
    this->stereo_calib.reset(new gtsam::Cal3_S2Stereo(this->camera_calib.camLeft.fx,
                                                this->camera_calib.camLeft.fy,
                                                0.00,
                                                this->camera_calib.camLeft.cx,
                                                this->camera_calib.camLeft.cy,
                                                this->camera_calib.extrinsic.tx));

    /** Optimized Output port **/
    this->vs_pose_out.invalidate();
    this->vs_pose_out.sourceFrame = _vsd_slam_localization_source_frame.value();

    /** Relative Frame to port out the SAM pose samples **/
    this->vs_pose_out.targetFrame = _world_frame.value();

    /** Odometry Output port **/
    this->odo_pose_out.invalidate();
    this->odo_pose_out.sourceFrame = _odometry_localization_source_frame.value();

    /** Relative Frame to port out the Odometry pose samples **/
    this->odo_pose_out.targetFrame = this->vs_pose_out.sourceFrame;

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[VSD_SLAM TASK] DESIRED TARGET FRAME IS: "<<vs_pose_out.targetFrame<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM TASK] STEREO CAMERA CALIBRATION PARAMETERS"<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM TASK] FX "<<this->camera_calib.camLeft.fx<<" FY "<< this->camera_calib.camLeft.fy <<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM TASK] CX "<<this->camera_calib.camLeft.cx<<" CY "<< this->camera_calib.camLeft.cy <<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM TASK] BASELINE "<< this->camera_calib.extrinsic.tx <<"\n"<<RTT::endlog();

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    /** Reset prediction filter **/
    this->filter.reset();

    /** Reset estimation **/
    this->pose_idx = 0;
    this->landmark_idx = 0;

    /** Reset GTSAM **/

}

void Task::initialization(Eigen::Affine3d &tf)
{
    /** The filter vector state variables for the navigation quantities **/
    WMTKState statek_0;

    /******************************/
    /** Initialize the Back-End  **/
    /******************************/

    /** Initial covariance matrix **/
    UKF::cov cov_statek_0; /** Initial P(0) for the state **/
    cov_statek_0.setZero();
    MTK::setDiagonal (cov_statek_0, &WMTKState::pos, 1e-10);
    MTK::setDiagonal (cov_statek_0, &WMTKState::orient, 1e-10);
    MTK::setDiagonal (cov_statek_0, &WMTKState::velo, 1e-12);
    MTK::setDiagonal (cov_statek_0, &WMTKState::angvelo, 1e-12);

    /***************************/
    /**  MTK initialization   **/
    /***************************/
    this->initUKF(statek_0, cov_statek_0);

    /***************************/
    /**  SAM initialization   **/
    /***************************/
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    gtsam::Pose3 first_pose(gtsam::Rot3(this->vs_pose_out.orientation), gtsam::Point3(this->vs_pose_out.position));

    /** Constrain the first pose such that it cannot change from its original value during optimization
    NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
    QR is much slower than Cholesky, but numerically more stable **/
    this->factor_graph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(frame_id, first_pose));

    /** Insert first pose in initial estimates **/
    this->sam_values.insert(frame_id, first_pose);

    /***************************************/
    /** Accumulate pose in MTK state form **/
    /***************************************/
    this->pose_state.pos = tf.translation(); //!Initial position
    this->pose_state.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Set the initial velocities in the state vector **/
    this->pose_state.velo.setZero(); //!Initial linear velocity
    this->pose_state.angvelo.setZero(); //!Initial angular velocity

    /** Accumulate pose in TWC form **/
    this->pose_with_cov.translation = tf.translation();
    this->pose_with_cov.orientation = this->pose_state.orient;

    return;
}

void Task::initUKF(WMTKState &statek, UKF::cov &statek_cov)
{
    /** Create the filter **/
    this->filter.reset (new UKF (statek, statek_cov));

    #ifdef DEBUG_PRINTS
    WMTKState state = this->filter->mu();
    RTT::log(RTT::Warning)<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM INIT UKF] State P0|0 is of size " <<this->filter->sigma().rows()<<" x "<<filter->sigma().cols()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM INIT UKF] State P0|0:\n"<<this->filter->sigma()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM INIT UKF] state:\n"<<state<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM INIT UKF] position:\n"<<state.pos<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM INIT UKF] orientation Roll: "<< base::getRoll(state.orient)*R2D
        <<" Pitch: "<< base::getPitch(state.orient)*R2D<<" Yaw: "<< base::getYaw(state.orient)*R2D<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM INIT UKF] velocity:\n"<<state.velo<<"\n";
    RTT::log(RTT::Warning)<<"[VSD_SLAM INIT UKF] angular velocity:\n"<<state.angvelo<<"\n";
    RTT::log(RTT::Warning)<< RTT::endlog();
    #endif


}

void Task::resetUKF(::base::Pose &current_delta_pose, ::base::Matrix6d &cov_current_delta_pose)
{

    /** Compute the delta pose since last time we reset the filter **/
    current_delta_pose.position = this->filter->mu().pos;// Delta position
    current_delta_pose.orientation = this->filter->mu().orient;// Delta orientation

    /** Compute the delta covariance since last time we reset the filter **/
    cov_current_delta_pose = this->filter->sigma().block<6,6>(0,0);

    /** Update the velocity in the pose state **/
    this->pose_state.velo = this->pose_state.orient * this->filter->mu().velo;// current linear velocity
    this->pose_state.angvelo = this->pose_state.orient * this->filter->mu().angvelo;// current angular velocity

    /** Reset covariance matrix **/
    UKF::cov P(UKF::cov::Zero());
    MTK::setDiagonal (P, &WMTKState::pos, 1e-10);
    MTK::setDiagonal (P, &WMTKState::orient, 1e-10);
    MTK::setDiagonal (P, &WMTKState::velo, 1e-12);
    MTK::setDiagonal (P, &WMTKState::angvelo, 1e-12);

    /** Remove the filter **/
    this->filter.reset();

    /** Create and reset a new filter **/
    WMTKState statek;
    this->initUKF(statek, P);

    return;
}

void Task::odo_poseOutputport(const base::Time &timestamp)
{
    WMTKState statek = this->filter->mu();

    /** Out port the last odometry pose **/
    this->odo_pose_out.position = statek.pos;
    this->odo_pose_out.cov_position = this->filter->sigma().block<3,3>(0,0);
    this->odo_pose_out.orientation = statek.orient;
    this->odo_pose_out.cov_orientation = this->filter->sigma().block<3,3>(3,3);
    this->odo_pose_out.velocity = statek.velo;
    this->odo_pose_out.cov_velocity =  this->filter->sigma().block<3,3>(6,6);
    this->odo_pose_out.angular_velocity = statek.angvelo;
    this->odo_pose_out.cov_angular_velocity =  this->filter->sigma().block<3,3>(9,9);
    _odo_pose_samples_out.write(this->odo_pose_out);

}
