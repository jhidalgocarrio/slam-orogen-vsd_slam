/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

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
    this->pose_idx = 0;
    this->landmark_idx = 0;

    this->init_flag = false;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
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
            RTT::log(RTT::Fatal)<<"[VS FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[VS POSE_SAMPLES] - Initializing Visual Stereo Back-End..."<<RTT::endlog();
        #endif

        /** Set initial pose out in body frame **/
        this->vs_pose.position = world_nav_tf.translation(); //!Initial position
        this->vs_pose.orientation = Eigen::Quaternion<double>(world_nav_tf.rotation());
        this->vs_pose.velocity.setZero(); //!Initial velocity
        this->vs_pose.angular_velocity.setZero(); //!Initial angular velocity

        /** Get the transformation Tbody_sensor **/
        Eigen::Affine3d body_sensor_tf; /** Transformer transformation **/
        if (_sensor_frame.value().compare(_body_frame.value()) == 0)
        {
            body_sensor_tf.setIdentity();
        }
        else if (!_sensor2body.get(ts, body_sensor_tf, false))
        {
            RTT::log(RTT::Fatal)<<"[VS FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        /** Store Tbody_sensor **/
        this->body_sensor_tf = body_sensor_tf;

        /** Change the initial pose out in sensor frame **/
        this->vs_pose.position = body_sensor_tf.inverse() * this->vs_pose.position; // p_sensor = Tsensor_body p_body
        this->vs_pose.orientation = this->vs_pose.orientation * Eigen::Quaternion <double>(body_sensor_tf.rotation()); //Tworld_sensor = Tworld_body * Tbody_sensor

        /***************************
        * BACK-END INITIALIZATION  *
        ***************************/
        Eigen::Affine3d vs_pose_tf = this->vs_pose.getTransform();
        this->initialization(vs_pose_tf);

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
        RTT::log(RTT::Fatal)<<"[VS FATAL ERROR]  No transformation provided."<<RTT::endlog();
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
}

void Task::visual_features_samplesTransformerCallback(const base::Time &ts, const ::visual_stereo::ExteroFeatures &visual_features_samples_sample)
{


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
    this->vs_pose.invalidate();
    this->vs_pose.sourceFrame = _vsd_slam_localization_source_frame.value();

    /** Relative Frame to port out the SAM pose samples **/
    this->vs_pose.targetFrame = _world_frame.value();

    /** Odometry Output port **/
    this->odo_pose.invalidate();
    this->odo_pose.sourceFrame = _odometry_localization_source_frame.value();

    /** Relative Frame to port out the Odometry pose samples **/
    this->odo_pose.targetFrame = this->vs_pose.sourceFrame;

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[VISUAL STEREO SLAM TASK] DESIRED TARGET FRAME IS: "<<vs_pose.targetFrame<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VISUAL STEREO SLAM TASK] STEREO CAMERA CALIBRATION PARAMETERS\n"<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VISUAL STEREO SLAM TASK] FX "<<this->camera_calib.camLeft.fx<<" FY "<< this->camera_calib.camLeft.fy <<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VISUAL STEREO SLAM TASK] CX "<<this->camera_calib.camLeft.cx<<" CY "<< this->camera_calib.camLeft.cy <<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VISUAL STEREO SLAM TASK] BASELINE "<< this->camera_calib.extrinsic.tx <<RTT::endlog();

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
    UKF::cov statek_cov; /** Initial P(0) for the state **/
    statek_cov.setZero();
    MTK::setDiagonal (statek_cov, &WMTKState::pos, 1e-10);
    MTK::setDiagonal (statek_cov, &WMTKState::orient, 1e-10);
    MTK::setDiagonal (statek_cov, &WMTKState::velo, 1e-12);
    MTK::setDiagonal (statek_cov, &WMTKState::angvelo, 1e-12);

    /***************************/
    /**  MTK initialization   **/
    /***************************/

    /** Create the filter **/
    this->filter.reset (new UKF (statek_0, statek_cov));

    /** Accumulate pose in MTK state form **/
    this->pose_state.pos = tf.translation(); //!Initial position
    this->pose_state.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Set the initial velocities in the state vector **/
    this->pose_state.velo.setZero(); //!Initial linear velocity
    this->pose_state.angvelo.setZero(); //!Initial angular velocity

    /** Accumulate pose in TWC form **/
    this->pose_with_cov.translation = tf.translation();
    this->pose_with_cov.orientation = this->pose_state.orient;

    /***************************/
    /**  SAM initialization   **/
    /***************************/
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    gtsam::Pose3 first_pose(gtsam::Rot3(this->vs_pose.orientation), gtsam::Point3(this->vs_pose.position));

    /** Constrain the first pose such that it cannot change from its original value during optimization
    NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
    QR is much slower than Cholesky, but numerically more stable **/
    this->factor_graph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(frame_id, first_pose));

    /** Insert first pose in initial estimates **/
    this->sam_values.insert(frame_id, first_pose);


    #ifdef DEBUG_PRINTS
    WMTKState state = this->filter->mu();
    RTT::log(RTT::Warning)<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VS INIT UKF] State P0|0 is of size " <<this->filter->sigma().rows()<<" x "<<filter->sigma().cols()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VS INIT UKF] State P0|0:\n"<<this->filter->sigma()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VS INIT UKF] state:\n"<<state<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VS INIT UKF] position:\n"<<state.pos<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VS INIT UKF] orientation Roll: "<< base::getRoll(state.orient)*R2D
        <<" Pitch: "<< base::getPitch(state.orient)*R2D<<" Yaw: "<< base::getYaw(state.orient)*R2D<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[VS INIT UKF] velocity:\n"<<state.velo<<"\n";
    RTT::log(RTT::Warning)<<"[VS INIT UKF] angular velocity:\n"<<state.angvelo<<"\n";
    RTT::log(RTT::Warning)<< RTT::endlog();
    #endif


    return;
}

