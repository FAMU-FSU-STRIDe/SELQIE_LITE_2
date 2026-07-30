#!/usr/bin/env python3

import os
import time
import rclpy
from cmd import Cmd

import rclpy.clock
from selqie_python.selqie import SELQIE

class SELQIETerminal(Cmd):
    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '

    def __init__(self):
        super().__init__()
        self._selqie = SELQIE()
        self._selqie.init()
        self._selqie.spin_background()

    def do_exit(self, line : str):
        """ Exit the terminal """
        print("Exiting...")
        self._selqie.stop()
        rclpy.shutdown()
        return True
    
    def do_idle(self, line : str):
        """ Idle the Cubemars motors """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_idle(i)
            
    def do_battery(self, line: str) -> None:
        """Print the latest battery voltage reading. Usage: battery"""
        if line.strip():
            print('Usage: battery')
            return

        voltage, stamp = self._selqie.snapshot_battery_voltage()
        if voltage is None or stamp is None:
            print('No battery voltage messages received yet.')
            return

        age_s = time.time() - stamp
        print(f'Battery voltage: {voltage:.2f} V (age {age_s:.1f}s)')

    def do_ready(self, line : str):
        """ Ready the Cubemars motors """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_ready(i)

    def do_clear_errors(self, line : str):
        """ Clear errors on the Cubemars motors """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_clear_errors(i)

    def do_zero(self, line : str):
        """ Set each Cubemars motor's current position to zero """
        _ = line
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_position_zero(i)

    def do_set_motor_position(self, line : str):
        """ Set the position of a motor """
        args = line.split()
        if len(args) != 2:
            print("Usage: set_motor_position <motor> <position>")
            return
        try:
            self._selqie.set_motor_position(int(args[0]), float(args[1]))
        except ValueError:
            print("Invalid motor or position values")

    # kp is stiffness, kd is damping. The driver applies, every cycle:
    #   torque = kp*(pos_err) + kd*(vel_err) + torq_ff
    # Protocol ranges: kp 0-500, kd 0-5 (the motor node clips anything outside).
    _GAIN_USAGE = "(kp 0-500 stiffness, kd 0-5 damping)"

    def do_set_gains(self, line : str):
        """ Set MIT gains on ALL motors, live -- no node restart:
            set_gains <kp> <kd> [velocity_kd] """
        args = line.split()
        if len(args) not in (2, 3):
            print(f"Usage: set_gains <kp> <kd> [velocity_kd]   {self._GAIN_USAGE}")
            return
        try:
            values = [float(a) for a in args]
        except ValueError:
            print("Invalid gain values")
            return

        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_gains(i, *values)
        shown = f"kp={values[0]} kd={values[1]}"
        if len(values) == 3:
            shown += f" velocity_kd={values[2]}"
        print(f"Set gains on all motors: {shown}")

    def do_set_motor_gains(self, line : str):
        """ Set MIT gains on ONE motor, live -- no node restart:
            set_motor_gains <motor> <kp> <kd> [velocity_kd] """
        args = line.split()
        if len(args) not in (3, 4):
            print(f"Usage: set_motor_gains <motor> <kp> <kd> [velocity_kd]   {self._GAIN_USAGE}")
            return
        try:
            motor = int(args[0])
            values = [float(a) for a in args[1:]]
        except ValueError:
            print("Invalid motor index or gain values")
            return

        try:
            self._selqie.set_motor_gains(motor, *values)
        except ValueError as e:
            print(e)
            return
        print(f"Set gains on motor {motor}: kp={values[0]} kd={values[1]}"
              + (f" velocity_kd={values[2]}" if len(values) == 3 else ""))

    def do_gains(self, line : str):
        """ Print the gains each motor is currently using """
        _ = line
        print(f"{'motor':>5}  {'kp':>8}  {'kd':>8}  {'velocity_kd':>12}")
        for i in range(self._selqie.NUM_MOTORS):
            g = self._selqie.get_motor_gains(i)
            if not g:
                # The node republishes gains at 1 Hz, so this only shows up if a
                # motor node is down or has not been discovered yet.
                print(f"{i:>5}  {'(no data)':>8}")
                continue
            vkd = f"{g[2]:.3f}" if len(g) > 2 else "-"
            print(f"{i:>5}  {g[0]:>8.3f}  {g[1]:>8.3f}  {vkd:>12}")

    def do_default(self, line : str):
        """ Keep launch-file motor gains and set default leg positions """
        for i in range(self._selqie.NUM_LEGS):
            self._selqie.set_leg_position_default(i)

    def do_set_leg_position(self, line : str):
        """ Set the position of a leg """
        args = line.split()
        if len(args) != 4:
            print("Usage: set_leg_position <leg_name/*> <x> <y> <z>")
            return
        try:
            leg = args[0]
            if leg == "*":
                for i in range(self._selqie.NUM_LEGS):
                    self._selqie.set_leg_position(i, float(args[1]), float(args[2]), float(args[3]))
            elif leg in self._selqie.LEG_NAMES:
                self._selqie.set_leg_position(self._selqie.LEG_NAMES.index(leg), float(args[1]), float(args[2]), float(args[3]))
            else:
                print("Invalid leg name")
        except ValueError:
            print("Invalid position values")

    def do_set_leg_force(self, line : str):
        """ Set the force of a leg """
        args = line.split()
        if len(args) != 4:
            print("Usage: set_leg_force <leg_name/*> <x> <y> <z>")
            return
        try:
            leg = args[0]
            if leg == "*":
                for i in range(self._selqie.NUM_LEGS):
                    self._selqie.set_leg_force(i, float(args[1]), float(args[2]), float(args[3]))
            elif leg in self._selqie.LEG_NAMES:
                self._selqie.set_leg_force(self._selqie.LEG_NAMES.index(leg), float(args[1]), float(args[2]), float(args[3]))
            else:
                print("Invalid leg name")
        except ValueError:
            print("Invalid force values")

    def _run_trajectory_loops(self, trajectories, num_loops : int, frequency : float):
        """ Publish the trajectory ONCE and let the publisher repeat it natively.

        Republishing once per cycle -- the previous approach -- was itself the
        problem: every trajectory message resets leg_trajectory_publisher_node
        to the start of the stride, so any jitter in the republish timing landed
        that reset mid-stride and snapped the foot back (a twitch). Publishing
        the 4 legs sequentially also meant their resets landed staggered, which
        is why a single leg could twitch on its own.

        Now the loop count travels with the message and the publisher repeats
        the stride in C++, advancing its anchor by exactly one period per
        repetition. Nothing republishes mid-run, so there is no reset to
        mistime, no per-cycle stagger between legs, and no dependence on Python
        or the ROS executor for motion timing. All 4 legs also share one stamped
        start instant, so they begin phase-aligned rather than at their own
        arrival times.

        This function is then only responsible for waiting out the run and
        reporting progress -- its timing no longer affects the motion at all.
        """
        period = self._selqie.get_leg_trajectory_period(trajectories)
        if period <= 0.0:
            # Fall back to the commanded frequency if the trajectory did not
            # carry a period (e.g. a hand-built message).
            period = 1.0 / frequency

        self._selqie.run_leg_trajectories(trajectories, loops=num_loops)

        # Wait out the run. This is progress reporting only -- the publisher is
        # already executing the whole sequence on its own clock -- but it must
        # not return early, or the caller would move on while the stride is
        # still playing. Playback begins one start-delay after the publish.
        start = time.monotonic() + self._selqie.TRAJECTORY_START_DELAY
        for loop_idx in range(num_loops):
            print(f"  Loop {loop_idx+1}/{num_loops}")
            remaining = (start + (loop_idx + 1) * period) - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

    def do_run_trajectory(self, line : str):
        """ Run a trajectory file or sequence of files """
        args = line.split()
        if len(args) % 3 != 0:
            print("Usage: run_trajectory <file1> <num_loops1> <frequency1> <file2> <num_loops2> <frequency2> ...")
            return
        try:
            for seg in range(0, len(args), 3):
                file = args[seg]
                num_loops = int(args[seg+1])
                frequency = float(args[seg+2])
                # Load/resample before creating the rate: this is the one-time,
                # non-negligible cost per file (parsing + resampling every leg).
                # If the rate were created first, that cost would eat straight
                # into loop 1's sleep budget -- only loop 1 sits in front of it,
                # since `trajectories` is computed once outside this loop -- so
                # loop 2 would fire early and restart the leg_trajectory_publisher
                # partway through loop 1 (visible as an early truncate-and-restart,
                # and as physical foot drag on whichever legs were in stance at
                # that moment). Loading first means the first full period starts
                # with zero setup debt, same as every loop after it.
                trajectories = self._selqie.get_leg_trajectories_from_file(file, frequency)
                print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
                self._run_trajectory_loops(trajectories, num_loops, frequency)
                print("Finished trajectory")
        except ValueError:
            print("Invalid number of loops or frequency")
        except FileNotFoundError:
            print("File not found")
            
    def complete_run_trajectory(self, text, line, begidx, endidx):
        """ Autocomplete for run_trajectory """
        if len(line.split()) % 3 == 1 or len(line.split()) % 3 == 2:
            files = os.listdir(self._selqie.TRAJECTORIES_FOLDER)
            return [f for f in files if f.startswith(text)]

    def do_run_trajectory_record(self, line : str):
        """ Run a trajectory file or sequence of files, recording a rosbag for the duration of the movement """
        args = line.split()
        if len(args) == 0 or len(args) % 3 != 0:
            print("Usage: run_trajectory_record <file1> <num_loops1> <frequency1> <file2> <num_loops2> <frequency2> ...")
            return
        if self._selqie.is_recording():
            print("Already recording; stop the current recording first")
            return
        try:
            specs = [(args[i], int(args[i+1]), float(args[i+2])) for i in range(0, len(args), 3)]
        except ValueError:
            print("Invalid number of loops or frequency")
            return

        tag = "+".join(
            f"{os.path.splitext(os.path.basename(file))[0]}_{frequency:g}Hz_{num_loops}x"
            for file, num_loops, frequency in specs
        )
        self._selqie.start_recording(tag)
        print(f"Started recording rosbag (tag: {tag})")
        try:
            for file, num_loops, frequency in specs:
                # See do_run_trajectory / _run_trajectory_loops: load before the
                # timing loop starts, and drive the loop off a monotonic deadline.
                trajectories = self._selqie.get_leg_trajectories_from_file(file, frequency)
                print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
                self._run_trajectory_loops(trajectories, num_loops, frequency)
                print("Finished trajectory")
        except ValueError:
            print("Invalid number of loops or frequency")
        except FileNotFoundError:
            print("File not found")
        finally:
            self._selqie.stop_recording()
            print("Stopped recording rosbag")

    def complete_run_trajectory_record(self, text, line, begidx, endidx):
        """ Autocomplete for run_trajectory_record """
        if len(line.split()) % 3 == 1 or len(line.split()) % 3 == 2:
            files = os.listdir(self._selqie.TRAJECTORIES_FOLDER)
            return [f for f in files if f.startswith(text)]

    def do_print_motor_info(self, line : str):
        """ Print motor info """
        for i in range(self._selqie.NUM_MOTORS):
            state = self._selqie.get_motor_estimate(i)
            err = self._selqie.get_motor_info(i)
            print(f"Motor {i}:")
            for attr in ["position", "abs_position", "velocity", "torque", "current", "temperature"]:
                print(f"  {attr}: {getattr(state, attr)}")
            print(f"  error: {err.data}")

    def do_print_leg_info(self, line : str):
        """ Print leg info """
        for i in range(self._selqie.NUM_LEGS):
            print(f"Leg {self._selqie.LEG_NAMES[i]}:")
            for attr in ["pos_estimate", "vel_estimate", "force_estimate"]:
                vector = getattr(self._selqie.get_leg_estimate(i), attr)
                print(f"  {attr}: x={vector.x}, y={vector.y}, z={vector.z}")

    def do_print_errors(self, line : str):
        """ Print all motor errors """
        iserr = False
        for i in range(self._selqie.NUM_MOTORS):
            err = self._selqie.get_motor_error_name(i)
            if err and "No fault" not in err:
                iserr = True
                print(f"Error on Motor {i}: {err}")
        if not iserr:
            print("No errors on all motors")

    def do_start_recording(self, line : str):
        """ Start rosbag recording of specific topics """
        self._selqie.start_recording()

    def do_stop_recording(self, line : str):
        """ Stop rosbag recording """
        self._selqie.stop_recording()
        
    def do_set_light_brightness(self, line : str):
        """ Set the brightness of the light """
        args = line.split()
        if len(args) != 1:
            print("Usage: set_light_brightness <brightness>")
            return
        try:
            self._selqie.set_vision_lights_brightness(float(args[0]))
        except ValueError:
            print("Invalid brightness value")
            return

    def do_set_led_color(self, line : str):
        """ Set the WS2812B LED color: set_led_color <r> <g> <b>  (0-255 each) """
        args = line.split()
        if len(args) != 3:
            print("Usage: set_led_color <r> <g> <b>")
            return
        try:
            r, g, b = int(args[0]), int(args[1]), int(args[2])
            if not all(0 <= v <= 255 for v in (r, g, b)):
                print("Values must be between 0 and 255")
                return
            self._selqie.set_led_color(r, g, b)
        except ValueError:
            print("Invalid color values")

    def do_led_off(self, line : str):
        """ Turn the WS2812B LED off """
        self._selqie.set_led_off()

    def do_latch_open(self, line : str):
        """ Open the latch servo """
        self._selqie.latch_open()

    def do_latch_close(self, line : str):
        """ Close the latch servo """
        self._selqie.latch_close()

    def do_set_gait(self, line : str):
        """ Set the gait for the robot """
        args = line.split()
        if len(args) != 1:
            print("Usage: set_gait <gait>")
            return
        try:
            if args[0] == "none":
                self._selqie.set_control_gait('')
            else:
                self._selqie.set_control_gait(args[0])
        except ValueError:
            print("Invalid gait")
            return
        
    def do_cmd_vel(self, line : str):
        """ Publish a Twist message to cmd_vel """
        args = line.split()
        if len(args) != 3:
            print("Usage: cmd_vel <lin_x> <lin_z> <ang_z>")
            return
        try:
            self._selqie.set_control_command_velocity(float(args[0]), float(args[1]), float(args[2]))
        except ValueError:
            print("Invalid values")
            return
        
    def do_walk(self, line : str):
        """ Walk the robot """
        args = line.split()
        if len(args)!= 2:
            print("Usage: walk <lin_x> <ang_z>")
            return
        try:
            self._selqie.set_control_gait('walk')
            time.sleep(0.1)
            self._selqie.set_control_command_velocity(float(args[0]), 0.0, float(args[1]))
        except ValueError:
            print("Invalid values")
            return
        
    def do_swim(self, line : str):
        """ Swim the robot """
        args = line.split()
        if len(args) != 2:
            print("Usage: swim <lin_x> <lin_z>")
            return
        try:
            self._selqie.set_control_gait('swim')
            time.sleep(0.1)
            self._selqie.set_control_command_velocity(float(args[0]), float(args[1]), 0.0)
        except ValueError:
            print("Invalid values")
            return
        
    def do_jump(self, line : str):
        """ Jump the robot """
        args = line.split()
        if len(args) != 2:
            print("Usage: jump <lin_x> <lin_z>")
            return
        try:
            self._selqie.set_control_gait('jump')
            time.sleep(0.1)
            self._selqie.set_control_command_velocity(float(args[0]), float(args[1]), 0.0)
        except ValueError:
            print("Invalid values")
            return
        
    def do_sink(self, line : str):
        """ Sink the robot """
        self._selqie.set_control_gait('sink')
        time.sleep(0.1)
        self._selqie.set_control_command_velocity(0.0, 0.0, 0.0)

    def do_stand(self, line : str):
        """ Stand the robot """
        self._selqie.set_control_gait('stand')
        time.sleep(0.1)
        self._selqie.set_control_command_velocity(0.0, 0.0, 0.0)

    def do_set_goal(self, line : str):
        """ Set the goal position for the robot """
        args = line.split()
        if len(args) != 3:
            print("Usage: set_goal <x> <y> <theta>")
            return
        try:
            self._selqie.set_control_goal_pose(float(args[0]), float(args[1]), float(args[2]))
        except ValueError:
            print("Invalid values")
            return
        
    def do_calibrate_imu(self, line : str):
        """ Calibrate the IMU """
        self._selqie.send_localization_calibrate_imu()

    def do_reset_localization(self, line : str):
        """ Reset the localization """
        self._selqie.set_localization_pose_zero()

    def do_reset_map(self, line : str):
        """ Reset the map """
        self._selqie.send_mapping_reset()
        
def main():
    rclpy.init()
    SELQIETerminal().cmdloop()

if __name__ == '__main__':
    main()
