#ifndef PIXHAWK_FSM_H
#define PIXHAWK_FSM_H
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <pixhawk_fsm/Explore.h>
#include <pixhawk_fsm/Land.h>
#include <pixhawk_fsm/OperationCompletion.h>
#include <pixhawk_fsm/TakeOff.h>
#include <pixhawk_fsm/Travel.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <mutex>

#include "operation.h"
#include "state_machine.h"
#include "status_publisher.h"

// structure to hold event data passed into state machine
struct Setpoint_Data : public EventData {
    geometry_msgs::Point point_of_interest;
    std::vector<geometry_msgs::Point> path;
    std::string target_operation;
};

/**
 * @brief Defines all the parameters for Pixhawk.
 */
struct PixhawkConfiguration {
    /**
     * @brief whether ekf is used or not
     */
    const bool ekf;

    /**
     * @brief whether use_perception is used or not
     */
    const bool use_perception;

    /**
     * @brief The unified refresh rate across the operation machine.
     */
    const int refresh_rate;

    /**
     * @brief Whether the drone will arm automatically.
     */
    const bool should_auto_arm;

    /**
     * @brief Whether the drone will go into offboard mode automatically.
     */
    const bool should_auto_offboard;

    /**
     * @brief Specifies the radius for within we can say that the drone is at a given position.
     */
    const float distance_completion_threshold;

    /**
     * @brief Specifies how low the velocity has to be before we issue that a given operation has
     * completed. This serves the purpose to prevent osciallations in the case when new operations
     * are fired the moment the previous completes. With this threshold, we have to wait for the
     * drone to be "steady" at the current position before moving on.
     */
    const float velocity_completion_threshold;

    /**
     * @brief Height used when e.g. 0 was given for a setpoint.
     */
    const float default_height;

    /**
     * @brief Show some debugging prints during the interact operation
     */
    const bool interaction_show_prints;

    /**
     * @brief Show some debugging prints during the interact operation
     */
    const float interact_max_vel;

    /**
     * @brief Use ground_truth data for interact operation.
     */
    const float interact_max_acc;

    /**
     * @brief max angle ardupilot parameter for the travel operation.
     */
    const float travel_max_angle;

    /**
     * @brief 3D offset of the Face_hugger compared to the drone center
     */
    const float* fh_offset;

    /**
     * @brief max speed ardupilot parameter for the travel operation.
     */
    const float travel_speed;

    /**
     * @brief max accel ardupilot parameter for the travel operation.
     */
    const float travel_accel;
};

class Pixhawk_fsm : public StateMachine {
   public:
    /**
     * @brief The configuration of the Pixhawk_fsm singleton.
     */
    const PixhawkConfiguration configuration;

    /**
     * @brief Initializes the Pixhawk_fsm singleton with a @p configuration.
     *
     * @param configuration The configuration of the Pixhawk_fsm singleton.
     *
     * @note Will only actually initialize if #instance_ptr is not nullptr.
     */
    static void initialize(const PixhawkConfiguration configuration);

    /**
     * @return The Pixhawk_fsm singleton instance.
     */
    static Pixhawk_fsm& getInstance();

    /**
     * @return The status publisher.
     */
    std::shared_ptr<StatusPublisher> getStatusPublisherPtr();

    /**
     * @brief Runs the operation macine.
     */
    void run();

   private:
    /**
     * @brief Only instance to this class.
     */
    static std::shared_ptr<Pixhawk_fsm> instance_ptr;

    /**
     * @brief Interface for publishing status messages.
     */
    std::shared_ptr<StatusPublisher> status_publisher_ptr;

    /**
     * @brief Sets up the service servers and clients.
     */
    Pixhawk_fsm(const PixhawkConfiguration configuration);

    /**
     * @brief Mapping for responses in service calls.
     */
    struct Response {
        /**
         * @brief Whether the service call was successful.
         */
        bool success;

        /**
         * @brief Error messsage (if any)
         */
        std::string message;
    };

    /**
     * @brief The current operation being executed
     */
    std::shared_ptr<Operation> current_operation_ptr;

    /**
     * @brief The current operation mutex to avoid data racing
     */
    std::unique_ptr<std::mutex> operation_ptr_mutex;

    /**
     * @brief The current operation being executed, essentially the current item in the list of
     * operations in #operation_execution_queue.
     */
    std::string current_operation;

    /**
     * @brief The list of operations which shall be executed.
     */
    std::list<std::shared_ptr<Operation>> operation_execution_queue;

    /**
     * @brief Flag for checking if one of the service handlers were called and that the FSM should
     * transition to another operation.
     */
    bool got_new_operation = false;

    /**
     * @brief Used to initialize the service servers.
     */
    ros::NodeHandle node_handle;

    /**
     * @brief The servers which advertise the operations.
     */
    ros::ServiceServer take_off_server, travel_server, kb_travel_server, explore_server,
        land_server;

    /**
     * @brief Used to give completion calls of operations.
     */
    ros::ServiceClient operation_completion_client;

    /**
     * @brief Service handler for the take off service.
     *
     * @param request The take off request.
     * @param response The take off response.
     *
     * @return true When the service call has been handled.
     */
    bool take_off(pixhawk_fsm::TakeOff::Request& request, pixhawk_fsm::TakeOff::Response& response);

    /**
     * @brief Service handler for the travel service.
     *
     * @param request The travel request.
     * @param response The travel response.
     *
     * @return true When the service call has been handled.
     */
    bool travel(pixhawk_fsm::Travel::Request& request, pixhawk_fsm::Travel::Response& response);

    /**
     * @brief Service handler for the kb_travel service.
     *
     * @param request The kb_travel request.
     * @param response The kb_travel response.
     *
     * @return true When the service call has been handled.
     */
    bool kb_travel(pixhawk_fsm::Travel::Request& request, pixhawk_fsm::Travel::Response& response);

    /**
     * @brief Service handler for the explore service.
     *
     * @param request The explore request.
     * @param response The explore response.
     *
     * @return true When the service call has been handled.
     */
    bool explore(pixhawk_fsm::Explore::Request& request, pixhawk_fsm::Explore::Response& response);

    /**
     * @brief Service handler for the land service.
     *
     * @param request The land request.
     * @param response The land response.
     *
     * @return true When the service call has been handled.
     */
    bool land(pixhawk_fsm::Land::Request& request, pixhawk_fsm::Land::Response& response);

    /**
     * @brief Will check if the operation to @p target_operation_identifier is valid and update the
     * #operation_execution_queue and #current_operation if it is.
     *
     * @param target_operation_identifier The target operation.
     * @param execution_queue The operation execution queue for the operation.
     *
     * @return Response based on the result of the attempt.
     */
    Response attemptToCreateOperation(const OperationIdentifier& target_operation_identifier,
                                      std::shared_ptr<Setpoint_Data> setPoint = NULL);

    /**
     * @brief Retrieves the operation identifier from @p operation_ptr
     *
     * @param operation_ptr The operation.
     *
     * @return operation identifier if operation_ptr is not nullptr, #OperationIdentifier::UNDEFINED
     * if else.
     */
    OperationIdentifier getOperationIdentifierForOperation(
        std::shared_ptr<Operation> operation_ptr);

    /**
     * @brief Performs operation transition between @p current_operation_ptr and @p
     * target_operation_ptr.
     *
     * @param current_operation_ptr The current operation.
     * @param target_operation_ptr The target operation.
     *
     * @return The new operation (@p target_operation_ptr).
     */
    std::shared_ptr<Operation> performOperationTransition(
        std::shared_ptr<Operation> current_operation_ptr,
        std::shared_ptr<Operation> target_operation_ptr);

    // external events taken by this state machine
    void Land(void);
    void Move(std::shared_ptr<Setpoint_Data> pData);

    // state machine state functions
    void ST_Idle(EventData*);
    void ST_Land(EventData*);
    void ST_Takeoff(std::shared_ptr<Setpoint_Data>);
    void ST_Move(std::shared_ptr<Setpoint_Data>);
    ;

    // state map to define state function order
    BEGIN_STATE_MAP
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Idle)
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Land)
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Takeoff)
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Move)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States { ST_IDLE = 0, ST_LAND, ST_TAKEOFF, ST_MOVE, ST_MAX_STATES };
};

#endif  // PIXHAWK_FSM_H