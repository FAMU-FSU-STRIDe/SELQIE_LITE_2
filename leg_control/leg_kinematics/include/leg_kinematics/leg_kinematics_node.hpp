#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

// Headers for custom ROS messages
#include <actuation_msgs/msg/motor_command.hpp>
#include <actuation_msgs/msg/motor_estimate.hpp>
#include <motor_interfaces/msg/motor_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <leg_control_msgs/msg/leg_command.hpp>
#include <leg_control_msgs/msg/leg_estimate.hpp>

// Use namespaces to simplify code
using namespace Eigen;
using namespace actuation_msgs::msg;
using namespace leg_control_msgs::msg;
using MotorState = motor_interfaces::msg::MotorState;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

/*
 * Fast Quality of Service Protocol
 * Messages can be dropped if neccessary to maintain performance
 * Used for high-frequency data like sensor readings
 */
static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

/*
 * Reliable Quality of Service Protocol
 * Ensures that messages are delivered reliably, even if it means delaying them
 * Used for critical commands or control messages where data loss is unacceptable
 */
static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

/*
 * Convert a Vector3f to a geometry_msgs::msg::Vector3 message
 */
static inline geometry_msgs::msg::Vector3 toVector3(const Vector3f &vec)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = vec(0);
    msg.y = vec(1);
    msg.z = vec(2);
    return msg;
}

/*
 * Convert a geometry_msgs::msg::Vector3 to a Vector3f
 */
static inline Vector3f fromVector3(const geometry_msgs::msg::Vector3 &msg)
{
    return Vector3f(msg.x, msg.y, msg.z);
}

/*
 * Abstract class for a Leg Kinematics Model
 */
class LegKinematicsModel
{
public:
    /*
     * Number of motors in the leg kinematics model.
     */
    virtual std::size_t get_num_motors() const = 0;

    /*
     * Forward kinematic function
     * Maps joint angles (motor positions) to the foot position in 3D space.
     */
    virtual Vector3f compute_forward_kinematics(const Vector3f &joint_angles) const = 0;

    /*
     * Inverse kinematic function
     * Maps a desired foot position in 3D space to the required joint angles (motor positions).
     */
    virtual Vector3f compute_inverse_kinematics(const Vector3f &foot_position) const = 0;

    /*
     * Jacobian matrix function
     * Computes the Jacobian matrix for the leg kinematics model.
     */
    virtual Matrix3f compute_jacobian_matrix(const Vector3f &joint_angles) const = 0;
};

/*
 * Leg Kinematics Node class
 */
class LegKinematicsNode
{
private:
    rclcpp::Node *_node;        // Pointer to the ROS node
    LegKinematicsModel *_model; // Pointer to the leg kinematics model

    rclcpp::Subscription<LegCommand>::SharedPtr _leg_command_sub; // Subscription for leg commands
    rclcpp::Publisher<LegEstimate>::SharedPtr _leg_estimate_pub;  // Publisher for leg estimates

    std::vector<rclcpp::Subscription<MotorEstimate>::SharedPtr> _motor_estimate_subs; // Subscriptions for legacy motor estimates
    std::vector<rclcpp::Subscription<MotorState>::SharedPtr> _motor_state_subs;        // Subscriptions for CubeMars motor states
    std::vector<rclcpp::Publisher<MotorCommand>::SharedPtr> _motor_command_pubs;       // Publishers for legacy motor commands
    std::vector<rclcpp::Publisher<Float64MultiArray>::SharedPtr> _motor_mit_cmd_pubs;   // Publishers for CubeMars MIT commands

    rclcpp::TimerBase::SharedPtr _motor_estimate_timer; // Timer for periodic leg estimate updates

    std::vector<MotorEstimate> _latest_motor_estimates; // Stores the latest motor estimates

    double _motor_kp; // CubeMars MIT position gain
    double _motor_kd; // CubeMars MIT velocity gain

    /*
     * Get the latest motor positions in vector form from the stored motor estimate messages
     */
    Vector3f _get_latest_motor_positions() const
    {
        // Zero-initialized: for models with fewer than 3 motors (e.g. the 2-motor
        // FiveBar2DModel used by all 4 legs), Eigen leaves unset entries uninitialized
        // otherwise, and the Jacobian's unused row/column maps that garbage straight
        // into the published foot estimate (e.g. vel_estimate.y / force_estimate.y).
        Vector3f positions = Vector3f::Zero();
        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            positions(i) = _latest_motor_estimates[i].pos_estimate;
        }
        return positions;
    }

    /*
     * Get the latest motor velocities in vector form from the stored motor estimate messages
     */
    Vector3f _get_latest_motor_velocities() const
    {
        Vector3f velocities = Vector3f::Zero();
        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            velocities(i) = _latest_motor_estimates[i].vel_estimate;
        }
        return velocities;
    }

    /*
     * Get the latest motor torques in vector form from the stored motor estimate messages
     */
    Vector3f _get_latest_motor_torques() const
    {
        Vector3f torques = Vector3f::Zero();
        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            torques(i) = _latest_motor_estimates[i].torq_estimate;
        }
        return torques;
    }

    bool _requires_jacobian(const LegCommand &msg) const
    {
        const Vector3f vel_setpoint = fromVector3(msg.vel_setpoint);
        const Vector3f force_setpoint = fromVector3(msg.force_setpoint);
        return msg.control_mode != LegCommand::CONTROL_MODE_POSITION ||
               vel_setpoint.squaredNorm() > 0.0f ||
               force_setpoint.squaredNorm() > 0.0f;
    }

    void _publish_motor_commands(
        const LegCommand &msg,
        const std::size_t motor_idx,
        const float motor_pos,
        const float motor_vel,
        const float motor_torq)
    {
        // Publish the legacy MotorCommand for simulation/ODrive-compatible nodes.
        MotorCommand motor_cmd;
        motor_cmd.control_mode = msg.control_mode; // control mode matches the leg control mode
        motor_cmd.input_mode = MotorCommand::INPUT_MODE_PASSTHROUGH;
        motor_cmd.pos_setpoint = motor_pos;
        motor_cmd.vel_setpoint = motor_vel;
        motor_cmd.torq_setpoint = motor_torq;
        _motor_command_pubs[motor_idx]->publish(motor_cmd);

        // Publish the CubeMars MIT command used by the hardware drivers:
        // [position, velocity, kp, kd, torque].
        Float64MultiArray mit_cmd;
        mit_cmd.data.resize(5);
        mit_cmd.data[0] = motor_pos;
        mit_cmd.data[1] = motor_vel;
        mit_cmd.data[2] = (msg.control_mode == LegCommand::CONTROL_MODE_POSITION) ? _motor_kp : 0.0;
        mit_cmd.data[3] = (msg.control_mode == LegCommand::CONTROL_MODE_FORCE) ? 0.0 : _motor_kd;
        mit_cmd.data[4] = motor_torq;
        _motor_mit_cmd_pubs[motor_idx]->publish(mit_cmd);
    }

    /*
     * Callback function for leg command messages
     */
    void _leg_command_callback(const LegCommand &msg)
    {
        // Convert message setpoints to eigen vectors
        const Vector3f pos_setpoint = fromVector3(msg.pos_setpoint);
        const Vector3f vel_setpoint = fromVector3(msg.vel_setpoint);
        const Vector3f force_setpoint = fromVector3(msg.force_setpoint);

        // Position-only trajectories do not need feedback-derived jacobians.  This
        // lets run_trajectory drive CubeMars hardware even when no legacy
        // motor/estimate publisher is present.
        const bool needs_jacobian = _requires_jacobian(msg);
        Matrix3f jacobian = Matrix3f::Identity();
        if (needs_jacobian)
        {
            jacobian = _model->compute_jacobian_matrix(_get_latest_motor_positions());

            // Make sure the jacobian is invertible
            if (jacobian.determinant() == 0.0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Jacobian determinant is zero");
                return;
            }
        }

        // Compute the motor positions, velocities, and torques
        const Vector3f motor_pos = _model->compute_inverse_kinematics(pos_setpoint);
        const Vector3f motor_vels = needs_jacobian ? jacobian.inverse() * vel_setpoint : Vector3f::Zero();
        const Vector3f motor_torqs = needs_jacobian ? jacobian.transpose() * force_setpoint : Vector3f::Zero();

        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            _publish_motor_commands(msg, i, motor_pos(i), motor_vels(i), motor_torqs(i));
        }
    }

    /*
     * Callback function for periodic leg estimate updates
     */
    void _leg_estimate_callback()
    {
        // Get the latest motor estimates
        const Vector3f motor_positions = _get_latest_motor_positions();

        // Compute the current jacobian matrix
        const Matrix3f jacobian = _model->compute_jacobian_matrix(motor_positions);

        // Make sure the jacobian is invertible
        if (jacobian.determinant() == 0.0)
        {
            RCLCPP_ERROR(_node->get_logger(), "Jacobian determinant is zero");
            return;
        }

        // Compute the foot position, velocity, and force estimates
        const Vector3f foot_position = _model->compute_forward_kinematics(motor_positions);
        const Vector3f foot_velocity = jacobian * _get_latest_motor_velocities();
        const Vector3f foot_force = jacobian.transpose().inverse() * _get_latest_motor_torques();

        // Create leg estimate message
        LegEstimate msg;
        msg.pos_estimate = toVector3(foot_position);
        msg.vel_estimate = toVector3(foot_velocity);
        msg.force_estimate = toVector3(foot_force);

        // Publish the leg estimate message
        _leg_estimate_pub->publish(msg);
    }

public:
    LegKinematicsNode(rclcpp::Node *node, LegKinematicsModel *model) : _node(node), _model(model)
    {
        const std::size_t num_motors = _model->get_num_motors();
        assert(num_motors > 0 && num_motors < 4); // Ensure the number of motors is valid (1 to 3)

        // Set the motor estimates vector size to the number of motors
        _latest_motor_estimates.resize(num_motors);

        node->declare_parameter("motor_kp", 50.0);
        node->declare_parameter("motor_kd", 0.05);
        _motor_kp = node->get_parameter("motor_kp").as_double();
        _motor_kd = node->get_parameter("motor_kd").as_double();

        // Create publisher and subscriber for leg commands and estimates
        _leg_command_sub = node->create_subscription<LegCommand>(
            "leg/command", qos_reliable(), std::bind(&LegKinematicsNode::_leg_command_callback, this, std::placeholders::_1));
        _leg_estimate_pub = node->create_publisher<LegEstimate>("leg/estimate", qos_fast());

        for (std::size_t m = 0; m < num_motors; m++)
        {
            // Create a callback function for each legacy motor estimate subscription
            const auto estimate_callback = [this, m](const MotorEstimate::SharedPtr msg)
            {
                _latest_motor_estimates[m] = *msg;
            };

            // Create a callback function for each CubeMars motor state subscription
            const auto state_callback = [this, m](const MotorState::SharedPtr msg)
            {
                _latest_motor_estimates[m].pos_estimate = msg->position;
                _latest_motor_estimates[m].vel_estimate = msg->velocity;
                _latest_motor_estimates[m].torq_estimate = msg->torque;
            };

            // Create the subscribers and publishers for each motor.  The legacy
            // command/estimate topics keep simulation support, while the
            // motor_state/mit_cmd topics connect to the CubeMars v2 hardware
            // driver used by SELQIE.
            _motor_estimate_subs.push_back(
                node->create_subscription<MotorEstimate>("motor" + std::to_string(m) + "/estimate", qos_fast(), estimate_callback));
            _motor_state_subs.push_back(
                node->create_subscription<MotorState>("motor" + std::to_string(m) + "/motor_state", qos_fast(), state_callback));
            _motor_command_pubs.push_back(
                node->create_publisher<MotorCommand>("motor" + std::to_string(m) + "/command", qos_reliable()));
            _motor_mit_cmd_pubs.push_back(
                node->create_publisher<Float64MultiArray>("motor" + std::to_string(m) + "/mit_cmd", qos_reliable()));
        }

        // Create a timer for periodic leg estimates
        node->declare_parameter("estimate_rate", 50.0);
        double estimate_rate = node->get_parameter("estimate_rate").as_double();
        _motor_estimate_timer = node->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / estimate_rate)),
            std::bind(&LegKinematicsNode::_leg_estimate_callback, this));

        RCLCPP_INFO(_node->get_logger(), "Leg Kinematics Node Initialized");
    }
};
