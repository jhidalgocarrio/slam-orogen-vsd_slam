/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VSD_SLAM_TASK_TASK_HPP
#define VSD_SLAM_TASK_TASK_HPP

#include "vsd_slam/TaskBase.hpp"

/** Rock libraries **/
#include <frame_helper/Calibration.h> /** Rock type for camera calibration parameters **/

/** Rock Types **/
#include <base/samples/RigidBodyState.hpp>
#include <base/TransformWithCovariance.hpp>

/** MTK **/
#include <mtk/types/pose.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/build_manifold.hpp>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>

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

/** GTSAM Optimizer **/
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace vsd_slam {

    /** MTK TYPES **/
    // We can't use types having a comma inside AutoConstruct macros :(
    typedef MTK::vect<3, double> vec3;
    typedef MTK::SO3<double> SO3;

    MTK_BUILD_MANIFOLD ( MTKState ,
    (( vec3, pos ))
    (( SO3, orient ))
    (( vec3, velo ))
    (( vec3, angvelo ))
    );

    typedef ukfom::mtkwrap<MTKState> WMTKState;
    typedef ukfom::ukf<WMTKState> UKF;

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
        unsigned long int pose_idx, landmark_idx;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Keys to identify poses and landmarks **/
        char pose_key, landmark_key;

        /** Intrinsic and extrinsic parameters for the pinhole camera model **/
        frame_helper::StereoCalibration camera_calib;

        /** GTSAM stereo calibration **/
        gtsam::Cal3_S2Stereo::shared_ptr stereo_calib;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** GTSAM Factor graph **/
        gtsam::NonlinearFactorGraph factor_graph;

        /** Values of the estimated quantities: TO-DO move to envire graph **/
        gtsam::Values sam_values;

        /** Filter for the pose prediction in an UT form **/
        boost::shared_ptr<UKF> filter;

        /** State of the MTK pre-integration filter **/
        WMTKState pose_state;

        /** State of the MTK pre-integration filter in Rock type **/
        base::TransformWithCovariance pose_with_cov;

        /**************************/
        /** Input port variables **/
        /**************************/

        /** Store the Tbody_sensor **/
        Eigen::Affine3d body_sensor_tf;

        /** Delta Pose estimation **/
        ::base::samples::RigidBodyState delta_pose;

        /***************************/
        /** Output port variables **/
        /***************************/
        base::samples::RigidBodyState vs_pose;
        base::samples::RigidBodyState odo_pose;

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

    };
}

#endif

