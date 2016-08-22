/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1
//#define DEBUG_EXECUTION_TIME 1

/** GTSAM Optimizer **/
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace vsd_slam;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    /** Set pose and landmark symbol identifiers **/
    this->pose_key = 'x';
    this->landmark_key = 'l';
    this->pose_idx = 0;

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    this->init_flag = false;

    /**************************/
    /** Input port variables **/
    /**************************/
    this->delta_pose.invalidate();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    /** Set pose and landmark symbol identifiers **/
    this->pose_key = 'x';
    this->landmark_key = 'l';
    this->pose_idx = 0;

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    this->init_flag = false;

    /**************************/
    /** Input port variables **/
    /**************************/
    this->delta_pose.invalidate();
}

Task::~Task()
{
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[VSD_SLAM DELTA_POSE_SAMPLES ] TIME: "<<delta_pose_samples_sample.time.toMicroseconds()<<RTT::endlog();
    #endif

    if (!this->init_flag)
    {
        /** Get the transformation Tworld_navigation **/
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

        Eigen::Affine3d body_sensor_tf; /** Transformer transformation **/
        /** Get the transformation Tbody_sensor **/
        if (_sensor_frame.value().compare(_body_frame.value()) == 0)
        {
            body_sensor_tf.setIdentity();
        }
        else if (!_sensor2body.get(ts, body_sensor_tf, false))
        {
            RTT::log(RTT::Fatal)<<"[VSD_SLAM FATAL ERROR] No transformation provided."<<RTT::endlog();
           return;
        }

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[VSD_SLAM DELTA_POSE_SAMPLES] - Initializing Visual Stereo Back-End..."<<RTT::endlog();
        #endif

        /***************************
        * BACK-END INITIALIZATION  *
        ***************************/
        Eigen::Affine3d init_tf (world_nav_tf * body_sensor_tf);
        this->initialization(init_tf);

        /** Initialization succeeded **/
        this->init_flag = true;

        /** Set the initial delta_pose **/
        this->delta_pose = delta_pose_samples_sample;

        /********************************
         ** Body Sensor initialization **
        *********************************/
        this->body_sensor_bs.initUnknown();
        this->body_sensor_bs.setPose(body_sensor_tf);

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[DONE]\n";
        #endif
    }

    /** Delta time between samples **/
    const double predict_delta_t = delta_pose_samples_sample.time.toSeconds() - this->delta_pose.time.toSeconds();
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[VSD_SLAM DELTA_POSE_SAMPLES] predict_delta_time: "<<predict_delta_t<<RTT::endlog();
    #endif

    /** A new sample arrived to the input port **/
    this->delta_pose = delta_pose_samples_sample;

    Eigen::Affine3d body_sensor_tf; /** Transformer transformation **/
    /** Get the transformation Tbody_sensor **/
    if (_sensor_frame.value().compare(_body_frame.value()) == 0)
    {
        body_sensor_tf.setIdentity();
    }
    else if (!_sensor2body.get(ts, body_sensor_tf, false))
    {
        RTT::log(RTT::Fatal)<<"[VSD_SLAM FATAL ERROR] No transformation provided."<<RTT::endlog();
       return;
    }

    /******************************************
    * Delta pose integration in sensor frame *
    * ****************************************/
    #ifdef DEBUG_EXECUTION_TIME
    clock_t start = clock();
    #endif

    /** Delta pose in sensor frame **/
    /** Ts(k-1)_s(k) = Ts(k-1)_b(k-1) * Tb(k-1)_b(k) * Tb(k)_s(k) **/
    this->delta_pose.setPose(this->body_sensor_bs.getPose().inverse() * this->delta_pose.getPose() * body_sensor_tf);
    this->delta_pose.linear_velocity() = this->delta_pose.orientation() * this->delta_pose.linear_velocity();
    this->delta_pose.angular_velocity() = this->delta_pose.orientation() * this->delta_pose.angular_velocity();
    /** TO-DO: what happed with the uncertainty since body_sensor_tf does not have uncertainty information **/

    /** Cumulative delta pose **/
    this->cumulative_delta_pose = this->cumulative_delta_pose * this->delta_pose;

    #ifdef DEBUG_EXECUTION_TIME
    clock_t end = clock();
    double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    RTT::log(RTT::Warning)<<"[VSD_SLAM DELTA_POSE_SAMPLES] execution time: "<<base::Time::fromMicroseconds(cpu_time_used*1000000.00)<<RTT::endlog();
    #endif

    /** Store Tbody_sensor **/
    this->body_sensor_bs.setPose(body_sensor_tf);

    /******************************************
    * Output port the odometry pose
    ******************************************/
    this->odo_poseOutputPort(this->delta_pose.time);
}

void Task::visual_features_samplesTransformerCallback(const base::Time &ts, const ::visual_stereo::ExteroFeatures &visual_features_samples_sample)
{

    /*********************************************/
    /** Check whether the Initialization is set **/
    /**********************************************/
    if(!init_flag)
    {
        RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] TASK STILL NOT INITIALIZED"<<RTT::endlog();
        return;
    }

    /****************************************
    ** Increase in one unit the pose index **
    ****************************************/
    this->pose_idx++;

    /****************************************************
    **   Store the delta pose in the factor graph     **
    ****************************************************/

    /** Symbols **/
    gtsam::Symbol symbol_prev = gtsam::Symbol(this->pose_key, this->pose_idx-1);
    gtsam::Symbol symbol_current = gtsam::Symbol(this->pose_key, this->pose_idx);

    /** Compute variance **/
    base::Matrix6d cov_cumulative_delta_pose;
    cov_cumulative_delta_pose << this->cumulative_delta_pose.cov_position(), Eigen::Matrix3d::Zero(),
                              Eigen::Matrix3d::Zero(), this->cumulative_delta_pose.cov_orientation();

    Eigen::SelfAdjointEigenSolver<base::Matrix6d> ev(cov_cumulative_delta_pose);
    base::Vector6d var_cumulative_delta_pose = ev.eigenvalues();


    /** Add the delta pose to the factor graph. TO-DO: probably not needed **/
    /**  BetweenFactor in GTSAM **/
    if (this->pose_idx == 1)
    {
        RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] BETWEEN_FACTOR: "<<std::string(symbol_prev)
            <<" -> "<< std::string(symbol_current)<<RTT::endlog();

        this->factor_graph->add(gtsam::BetweenFactor<gtsam::Pose3>(symbol_prev, symbol_current,
                gtsam::Pose3(gtsam::Rot3(this->cumulative_delta_pose.orientation()), gtsam::Point3(this->cumulative_delta_pose.position())),
                gtsam::noiseModel::Diagonal::Variances(var_cumulative_delta_pose)));
    }

    /***********************************************
     * Add the cumulative delta pose to the pose
    ***********************************************/

    /** Compute the pose estimate **/
    this->pose_with_cov =  this->pose_with_cov * this->cumulative_delta_pose;

    /***********************************************
    * Store Pose estimated values in GTSAM
    * **********************************************/
    RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] ESTIMATE VALUE: "<<std::string(symbol_current)<<RTT::endlog();
    gtsam::Pose3 current_pose(gtsam::Rot3(this->pose_with_cov.orientation()), gtsam::Point3(this->pose_with_cov.position()));
    this->estimate_values->insert(symbol_current, current_pose);

    /****************************************************/
    /** Reset the accumulated delta pose **/
    /****************************************************/
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] CUMULATIVE DELTA POSE\n"<<this->cumulative_delta_pose<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] RESET CUMULATIVE\n";
    #endif

    this->cumulative_delta_pose.initUnknown();
    base::Matrix6d cov; cov.setIdentity(); cov *= 1e-10;
    this->pose_with_cov.pose.setCovariance(cov);
    this->pose_with_cov.velocity.setCovariance(cov);


    const int start_position = 18;//can vary - we can use rand() too
    const int length_hex = 14;//can vary - we can use rand() too

    /******************************************************
    ** Read Stereo measurement from the input port 
    ******************************************************/
    gtsam::Symbol image_symbol = gtsam::Symbol(this->landmark_key, visual_features_samples_sample.img_idx);
    RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] IMAGE ID: "<<std::string(image_symbol)<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] RECEIVED  "<<visual_features_samples_sample.features.size()<<" SAMPLES"<<RTT::endlog();
    for (std::vector<visual_stereo::Feature>::const_iterator it = visual_features_samples_sample.features.begin();
            it != visual_features_samples_sample.features.end(); ++it)
    {
        /** Get the feature index **/
        std::string index_str = boost::uuids::to_string((*it).index);
        RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] FEATURE ID STRING: "<<index_str<<RTT::endlog();
        std::remove( index_str.begin(), index_str.end(), '-');
        index_str = "0x" + index_str.substr(start_position, length_hex);
        std::uint64_t index_uint = std::stoull(index_str, 0, 16); //random out of UUID
        gtsam::Symbol feature_symbol = gtsam::Symbol(this->landmark_key, index_uint);
        //RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] FEATURE ID: "<< std::string(feature_symbol)<<RTT::endlog();

        /** Get the feature stereo point **/
        base::Vector3d const &stereo_point((*it).stereo_point);

        /** Noise model of pixel coordinates **/
        const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3,1);

        /******************************************************
        * Set Generic Stereo Factor and features pose in GTSAM
        ******************************************************/
        RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] STEREO_FACTOR: "<<std::string(symbol_current)
            <<" -> "<< std::string(feature_symbol)<<RTT::endlog();
        this->factor_graph->push_back(
                gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(
                    gtsam::StereoPoint2(stereo_point[0], stereo_point[1], stereo_point[2]),
                    model, symbol_current, feature_symbol, this->stereo_calib));

        /******************************************************
         * Set the Feature initial estimated position
        ******************************************************/
        if (!this->estimate_values->exists(feature_symbol))
        {
            base::Vector3d feature_position_base = this->pose_with_cov.getPose() * (*it).point_3d; // p_navigation_frame = Tnav_sensor_frame * Tp_sensor_frame
            RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES ] FEATURE POSE["<< std::string(feature_symbol) <<"]: "<<
                feature_position_base[0]<<" "<<feature_position_base[1]<<" "<<feature_position_base[2]<<RTT::endlog();
            gtsam::Point3 feature_position (feature_position_base);
            this->estimate_values->insert(feature_symbol, feature_position);
        }
    }

    /********************************
    ** Optimize
    ********************************/
    if ((visual_features_samples_sample.img_idx % 50) == 0)
    {
        this->optimize();
    }

    /********************************
    ** Output port the slam pose **
    ********************************/
    this->slam_poseOutputPort(visual_features_samples_sample.time, symbol_current);

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"********************************************\n";
    RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES] CURRENT POSITION:\n"<<this->pose_with_cov.position()<<"\n";
    RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURESM] CURRENT ORIENTATION ROLL: "<< base::getRoll(this->pose_with_cov.orientation())*R2D
        <<" PITCH: "<< base::getPitch(this->pose_with_cov.orientation())*R2D<<" YAW: "<< base::getYaw(this->pose_with_cov.orientation())*R2D<<std::endl;
    //RTT::log(RTT::Warning)<<"[VSD_SLAM FEATURES] CURRENT COVARIANCE:\n"<<this->pose_with_cov.cov<<"\n";
    #endif
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
    this->slam_pose_out.invalidate();
    this->slam_pose_out.sourceFrame = _slam_localization_source_frame.value();

    /** Relative Frame to port out the SAM pose samples **/
    this->slam_pose_out.targetFrame = _world_frame.value();

    /** Odometry Output port **/
    this->odo_pose_out.invalidate();
    this->odo_pose_out.sourceFrame = _odometry_localization_source_frame.value();

    /** Relative Frame to port out the Odometry pose samples **/
    this->odo_pose_out.targetFrame = this->slam_pose_out.sourceFrame;

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[VSD_SLAM TASK] DESIRED TARGET FRAME IS: "<<this->slam_pose_out.targetFrame<<RTT::endlog();
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

    /** Reset GTSAM **/
    this->factor_graph.reset();
    this->estimate_values.reset();

    /** Reset estimation **/
    this->pose_idx = 0;
}

void Task::initialization(Eigen::Affine3d &tf)
{
    /**********************************************
    **  Cumulative delta pose initialization
    ***********************************************/
    this->cumulative_delta_pose.initUnknown();
    base::Matrix6d cov; cov.setIdentity(); cov *= 1e-10;
    this->cumulative_delta_pose.pose.setCovariance(cov);
    this->cumulative_delta_pose.velocity.setCovariance(cov);

    /***************************/
    /**    Initialization     **/
    /***************************/
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    gtsam::Pose3 first_pose(gtsam::Rot3(tf.rotation()), gtsam::Point3(tf.translation()));

    /** Create the factor graph **/
    this->factor_graph.reset(new gtsam::NonlinearFactorGraph());

    /** Constrain the first pose such that it cannot change from its original value during optimization
    NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
    QR is much slower than Cholesky, but numerically more stable **/
    this->factor_graph->push_back(gtsam::NonlinearEquality<gtsam::Pose3>(frame_id, first_pose));

    /** Create the estimated values **/
    this->estimate_values.reset(new gtsam::Values());

    /** Insert first pose in initial estimates **/
    this->estimate_values->insert(frame_id, first_pose);
    RTT::log(RTT::Warning)<<"[VSD_SLAM INITIALIZATION ] INITIAL POSE ID: "<<std::string(frame_id)<<RTT::endlog();

    /*************************
    ** Pose initialization  **
    *************************/
    this->pose_with_cov.setPose(tf);
    this->pose_with_cov.pose.setCovariance(cov);
    this->pose_with_cov.velocity.setVelocity(base::Vector6d::Zero());
    this->pose_with_cov.velocity.setCovariance(cov);

    return;
}

void Task::optimize()
{
    gtsam::LevenbergMarquardtParams params;
    params.orderingType = gtsam::Ordering::METIS;
    gtsam::LevenbergMarquardtOptimizer optimizer = gtsam::LevenbergMarquardtOptimizer(*(this->factor_graph), *(this->estimate_values), params);

    /** Store in the values **/
    this->estimate_values.reset(new gtsam::Values(optimizer.optimize()));
    RTT::log(RTT::Warning)<<"[VSD_SLAM OPTIMIZE] ESTIMATE_VALUES WITH: "<<this->estimate_values->size()<<"\n";

    return;
}

void Task::odo_poseOutputPort(const base::Time &timestamp)
{
    /** Out port the last odometry pose **/
    this->odo_pose_out.time = timestamp;
    this->odo_pose_out.position = this->cumulative_delta_pose.position();
    this->odo_pose_out.cov_position = this->cumulative_delta_pose.cov_position();
    this->odo_pose_out.orientation = this->cumulative_delta_pose.orientation();
    this->odo_pose_out.cov_orientation = this->cumulative_delta_pose.cov_orientation();
    this->odo_pose_out.velocity = this->cumulative_delta_pose.linear_velocity();
    this->odo_pose_out.cov_velocity =  this->cumulative_delta_pose.cov_linear_velocity();
    this->odo_pose_out.angular_velocity = this->cumulative_delta_pose.angular_velocity();
    this->odo_pose_out.cov_angular_velocity =  this->cumulative_delta_pose.cov_angular_velocity();
    _odo_pose_samples_out.write(this->odo_pose_out);

}

void Task::slam_poseOutputPort(const base::Time &timestamp, const gtsam::Symbol &symbol)
{
    /** Get the pose **/
    const gtsam::Pose3& last_pose = this->estimate_values->at<gtsam::Pose3>(symbol);

    /** Out port the last slam pose **/
    this->slam_pose_out.time = timestamp;
    this->slam_pose_out.position = last_pose.translation().vector();
    this->slam_pose_out.orientation = last_pose.rotation().toQuaternion();
    this->slam_pose_out.velocity = this->pose_with_cov.linear_velocity();
    this->slam_pose_out.cov_velocity =  this->pose_with_cov.cov_linear_velocity();
    this->slam_pose_out.angular_velocity = this->pose_with_cov.angular_velocity();
    this->slam_pose_out.cov_angular_velocity =  this->pose_with_cov.cov_angular_velocity();
    _pose_samples_out.write(this->slam_pose_out);

//    this->pose_with_cov  = this->slam_pose_out;

}
