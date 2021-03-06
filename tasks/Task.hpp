/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VSD_SLAM_TASK_TASK_HPP
#define VSD_SLAM_TASK_TASK_HPP

#include "vsd_slam/TaskBase.hpp"

/** STD **/
#include <vector>
#include <cstdlib>
#include <cmath>
#include <time.h>
#include <stdlib.h>

/** Boost **/
#include <boost/shared_ptr.hpp> /** shared pointers **/
#include <boost/uuid/uuid_io.hpp>

/** Eigen **/
#include <Eigen/Core> /** Core */
#include <Eigen/StdVector> /** For STL container with Eigen types **/

/** GTSAM TYPES **/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

/** GTSAM Factors **/
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/StereoFactor.h>


/** Base Types **/
#include <base/samples/BodyState.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

/** Rock libraries **/
#include <frame_helper/Calibration.h> /** Rock type for camera calibration parameters **/

namespace vsd_slam {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the vsd_slam namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','vsd_slam::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Structures having Eigen members

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/
        bool init_flag;

        /** Indices to identify poses and landmarks **/
        unsigned long int pose_idx;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Keys to identify poses and landmarks **/
        char pose_key, landmark_key;

        /** Intrinsic and extrinsic parameters for the pinhole camera model **/
        frame_helper::StereoCalibration camera_calib;

        /** GTSAM stereo calibration **/
        gtsam::Cal3_S2Stereo::shared_ptr stereo_calib;

        /**************************/
        /** Input port variables **/
        /**************************/

        /** Delta Pose estimation **/
        ::base::samples::BodyState delta_pose;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** GTSAM Factor graph **/
        boost::shared_ptr<gtsam::NonlinearFactorGraph> factor_graph;

        /** Values of the estimated quantities: TO-DO move to envire graph **/
        boost::shared_ptr<gtsam::Values> estimate_values;

        /** Cumulative delta pose between features samples  **/
        base::samples::BodyState cumulative_delta_pose;

        /** Pre-integration pose with covariance **/
        base::samples::BodyState pose_with_cov;

        /** Transformation between body and sensor frame **/
        base::samples::BodyState body_sensor_bs;

        /***************************/
        /** Output port variables **/
        /***************************/
        base::samples::RigidBodyState slam_pose_out;
        base::samples::RigidBodyState odo_pose_out;

    protected:

        virtual void delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample);

        virtual void visual_features_samplesTransformerCallback(const base::Time &ts, const ::visual_stereo::ExteroFeatures &visual_features_samples_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "vsd_slam::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /**@brief initialization
         */
        void initialization(Eigen::Affine3d &tf);

        /**@brief Output port the odometry pose
         */
        void odo_poseOutputPort(const base::Time &timestamp);

        /**@brief Output port the slam pose
        * */
        void slam_poseOutputPort(const base::Time &timestamp, const gtsam::Symbol &symbol);

        /** @brief Optimize
         * */
        void optimize();
    };
}

#endif

