#include <rclcpp/rclcpp.hpp>
#include <leg_control_msgs/msg/leg_command.hpp>
#include <leg_control_msgs/msg/leg_trajectory.hpp>

// Leg command poll rate in nanoseconds
// This is the maximum rate at which leg commands can be generated and sent to the legs
#define LEG_COMMAND_POLL_RATE_NS 1000000UL

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

// Use namespaces to avoid long names
using namespace leg_control_msgs::msg;

/*
 * LegTrajectoryPublisherNode
 * This node subscribes to a leg trajectory and publishes leg commands
 */
class LegTrajectoryPublisherNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<LegTrajectory>::SharedPtr _leg_trajectory_sub; // Subscription to leg trajectory
    rclcpp::Publisher<LegCommand>::SharedPtr _leg_command_pub;          // Publisher for leg commands
    rclcpp::TimerBase::SharedPtr _timer;                                // Timer for publishing leg commands

    std::size_t _idx = 0;           // Current index in the trajectory
    LegTrajectory::SharedPtr _traj; // Pointer to the current leg trajectory
    double _start_time;       // Start time of the current repetition

    uint32_t _loops_total = 1;   // Repetitions requested (1 = play once)
    uint32_t _loops_done = 0;    // Repetitions completed so far
    double _period = 0.0;        // Cycle duration used to schedule repeats

    /*
     * Derive the cycle duration from the timing array.
     * The samples span [t_first, t_last], which is one sample-step short of a
     * full cycle: the next repetition's first sample belongs one step after
     * t_last. Adding the average step back recovers the true period, so that
     * repeating the trajectory is seamless rather than losing a step each time.
     */
    static double _derive_period(const std::vector<double> &timing)
    {
        const std::size_t n = timing.size();
        if (n < 2)
        {
            return 0.0;
        }
        const double span = timing.back() - timing.front();
        return span + (span / static_cast<double>(n - 1));
    }

    /*
     * Trajectory callback function
     */
    void _trajectory_callback(const LegTrajectory::SharedPtr msg)
    {
        // Use the timing data to activate or deactivate the node
        if (!msg->timing.empty())
        {
            // If not empty, update the trajectory

            // Check if the trajectory is valid
            if (msg->timing.size() != msg->commands.size())
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory size mismatch: %lu timing, %lu commands",
                             msg->timing.size(), msg->commands.size());
                return;
            }

            // Check if the trajectory is in chronological order
            if (!std::is_sorted(msg->timing.begin(), msg->timing.end()))
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory timing not in chronological order");
                return;
            }

            // Reset the trajectory index and start time
            _idx = 0;
            _traj = msg;

            // Anchor playback. An explicit start_time lets every leg share one
            // phase reference even though their messages arrive at slightly
            // different times; without it, fall back to the arrival time.
            _start_time = (msg->start_time > 0.0) ? msg->start_time : this->now().seconds();

            // Repeat natively rather than making the sender republish per cycle.
            _loops_total = (msg->loops == 0) ? 1 : msg->loops;
            _loops_done = 0;
            _period = (msg->period > 0.0) ? msg->period : _derive_period(msg->timing);

            // Without a usable period there is nothing to schedule repeats
            // against, so fall back to playing the trajectory once.
            if (_loops_total > 1 && _period <= 0.0)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Cannot repeat trajectory %u times without a period; playing once",
                            _loops_total);
                _loops_total = 1;
            }

            // Activate the node if not already active
            if (!_timer)
            {
                // Create a timer to publish leg commands at the specified rate
                _timer = this->create_wall_timer(
                    std::chrono::nanoseconds(LEG_COMMAND_POLL_RATE_NS),
                    std::bind(&LegTrajectoryPublisherNode::_publish_leg_command, this));
            }
        }
        else
        {
            // If empty, deactivate the node
            if (_timer)
            {
                // Cancel the timer
                _timer->cancel();

                // Reset the shared pointer to the timer
                _timer.reset();
            }
        }
    }

    /*
     * Publish leg command function
     * This function is called at a fixed rate to publish leg commands
     */
    void _publish_leg_command()
    {
        const auto current_time = this->now().seconds();
        if (current_time < _start_time)
        {
            // If the current time is before the start time, return
            // Happens if the simulation is reset
            return;
        }

        const auto diff = current_time - _start_time;

        // Advance past every command whose scheduled time has already elapsed and
        // publish only the most recent one. Advancing a single index per timer tick
        // capped playback at one command per tick (~1 kHz): when a trajectory is
        // replayed at a higher frequency its timeline is compressed below that, so
        // the publisher fell behind and the next republish truncated the stride
        // ("cut short") while the motion speed plateaued. Skipping to the setpoint
        // due *now* keeps pace with the compressed timeline so speed scales with
        // frequency. Skipped intermediate setpoints are stale positions the leg is
        // already moving through, so only the latest is commanded.
        std::size_t next = _idx;
        while (next < _traj->timing.size() && _traj->timing[next] <= diff)
        {
            next++;
        }

        // Nothing new is due yet; wait for the next tick.
        if (next == _idx)
        {
            return;
        }

        // Publish the latest command whose time has elapsed.
        _leg_command_pub->publish(_traj->commands[next - 1]);
        _idx = next;

        // Check if this repetition has been fully played out
        if (_idx >= _traj->timing.size())
        {
            _loops_done++;

            if (_loops_done < _loops_total)
            {
                // Start the next repetition. Advancing the anchor by exactly one
                // period (rather than re-anchoring to "now") keeps repetitions
                // phase-exact and stops scheduling slop from accumulating across
                // a long run. Rewinding the index here is a seamless wrap, not a
                // mid-stride reset, because it happens only once the previous
                // repetition has actually finished.
                _idx = 0;
                _start_time += _period;
                return;
            }

            // All repetitions done: deactivate the trajectory publisher

            // Cancel the timer
            _timer->cancel();

            // Reset the shared pointer to the timer
            _timer.reset();
        }
    }

public:
    LegTrajectoryPublisherNode() : rclcpp::Node("leg_trajectory_publisher")
    {
        // Create subscriptions and publishers
        _leg_trajectory_sub = this->create_subscription<LegTrajectory>(
            "leg/trajectory", qos_reliable(),
            std::bind(&LegTrajectoryPublisherNode::_trajectory_callback, this, std::placeholders::_1));

        _leg_command_pub = this->create_publisher<LegCommand>("leg/command", qos_reliable());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegTrajectoryPublisherNode>());
    rclcpp::shutdown();
    return 0;
}