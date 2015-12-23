/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace visual_stereo;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    this->pose_idx = 0;
    this->landmark_idx = 0;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    this->pose_idx = 0;
    this->landmark_idx = 0;
}

Task::~Task()
{
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{

    if (!this->initialization)
    {
        Eigen::Affine3d world_nav_tf; /** Transformer transformation **/

        /** Get the transformation Tworld_navigation (navigation is body_0) **/
        if (_navigation_frame.value().compare(_world_frame.value()) == 0)
        {
            world_nav_tf.setIdentity();
        }
        else if (!_navigation2world.get(ts, world_nav_tf, false))
        {
           RTT::log(RTT::Fatal)<<"[SAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[SAM POSE_SAMPLES] - Initializing Filter..."<<RTT::endlog();
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
           RTT::log(RTT::Fatal)<<"[SAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        /** Store Tbody_sensor **/
        this->body_sensor_tf = body_sensor_tf;

        /** Change the initial pose out in sensor frame **/
        this->vs_pose.position = body_sensor_tf.inverse() * this->vs_pose.position; // p_sensor = Tsensor_body p_body
        this->vs_pose.orientation = this->vs_pose.orientation * Eigen::Quaternion <double>(body_sensor_tf.rotation()); //Tworld_sensor = Tworld_body * Tbody_sensor

        /** SAM initialization **/
        gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
        gtsam::Pose3 first_pose(gtsam::Rot3(this->vs_pose.orientation), gtsam::Point3(this->vs_pose.position));

        /** Constrain the first pose such that it cannot change from its original value during optimization
        NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
        QR is much slower than Cholesky, but numerically more stable **/
        this->factor_graph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(frame_id, first_pose));

        /** Insert first pose in initial estimates **/
        this->sam_values.insert(frame_id, first_pose);

        this->pose_idx++;

        this->initialization = true;

        #ifdef DEBUG_PRINTS
        //RTT::log(RTT::Warning)<<"[DONE]\n";
        #endif
    }

    Eigen::Affine3d body_sensor_tf; /** Transformer transformation **/

    /** Get the transformation Tbody_sensor **/
    if (_sensor_frame.value().compare(_body_frame.value()) == 0)
    {
        body_sensor_tf.setIdentity();
    }
    else if (!_sensor2body.get(ts, body_sensor_tf, false))
    {
       RTT::log(RTT::Fatal)<<"[SAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
       return;
    }

    /** Calculate the delta pose Tb(k-1)_b(k) **/
    Eigen::Affine3d body_delta_pose_tf (delta_pose_samples_sample.orientation);
    body_delta_pose_tf.translation() = delta_pose_samples_sample.position;

    /** Calculate the delta pose in sensor(s) Ts(k-1)_s(k) **/
    /** Ts(k-1)_s(k) = Ts(k-1)_b(k-1) * Tb(k-1)_b(k) * Tb(k)_s(k) **/
    Eigen::Affine3d sensor_delta_pose_tf = this->body_sensor_tf.inverse() * body_delta_pose_tf * body_sensor_tf;

    /** Store the current delta_pose in sensor frame. Ts(k-1)_s(k) **/
    this->delta_pose.position = sensor_delta_pose_tf.translation();
    this->delta_pose.orientation =  Eigen::Quaternion <double>(sensor_delta_pose_tf.rotation());
    this->delta_pose.cov_position.setZero(); this->delta_pose.cov_orientation.setZero();
    this->delta_pose.cov_position.diagonal() = body_sensor_tf.rotation().inverse() * delta_pose_samples_sample.cov_position.diagonal();
    this->delta_pose.cov_orientation.diagonal() = body_sensor_tf.rotation().inverse() * delta_pose_samples_sample.cov_orientation.diagonal();

    /** Store Tbody_sensor **/
    this->body_sensor_tf = body_sensor_tf;

    /** Delta noise model **/
    ::base::Matrix6d cov_delta_pose;
    cov_delta_pose << this->delta_pose.cov_position, Eigen::Matrix3d::Zero(),
                   Eigen::Matrix3d::Zero(), this->delta_pose.cov_orientation;

    /** Symbols **/
    gtsam::Symbol symbol1 = gtsam::Symbol(this->pose_key, this->pose_idx-1);
    gtsam::Symbol symbol2 = gtsam::Symbol(this->pose_key, this->pose_idx);


    /** Add the delta pose to the factor graph **/
    this->factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symbol1, symbol2,
                gtsam::Pose3(gtsam::Rot3(this->delta_pose.orientation), gtsam::Point3(this->delta_pose.position)),
                gtsam::noiseModel::Gaussian::Covariance(cov_delta_pose)));


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
    this->vs_pose.sourceFrame = _visual_stereo_localization_source_frame.value();

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

    /** Reset estimation **/
    this->pose_idx = 0;
    this->landmark_idx = 0;

}
