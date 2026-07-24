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
        # The ready/stand move is a gentle hold -> use the smooth position-speed
        # submode. Gaits switch to plain position per their frequency below.
        self._selqie.set_all_motors_position_mode('pos_spd')

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

    def do_set_gains(self, line : str):
        """ Set the gains for the motors: set_gains <p_gain> <v_gain> """
        args = line.split()
        if len(args) != 2:
            print("Usage: set_gains <p_gain> <v_gain>")
            return
        try:
            for i in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(i, float(args[0]), float(args[1]))
        except ValueError:
            print("Invalid gain values")

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

    def do_run_trajectory(self, line : str):
        """ Run a trajectory file or sequence of files """
        args = line.split()
        if len(args) % 3 != 0:
            print("Usage: run_trajectory <file1> <num_loops1> <frequency1> <file2> <num_loops2> <frequency2> ...")
            return
        try:
            # Gaits run in plain position (SET_POS) for accurate tracking at any
            # frequency; only the stand/ready hold uses the position-speed submode.
            self._selqie.set_all_motors_position_mode('pos')
            time.sleep(0.05)  # let the mode change take effect before streaming
            for seg in range(0, len(args), 3):
                file = args[seg]
                num_loops = int(args[seg+1])
                frequency = float(args[seg+2])
                rate = self._selqie.create_rate(frequency)
                trajectories = self._selqie.get_leg_trajectories_from_file(file, frequency)
                print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
                for loop_idx in range(num_loops):
                    print(f"  Loop {loop_idx+1}/{num_loops}")
                    self._selqie.run_leg_trajectories(trajectories)
                    rate.sleep()
                print("Finished trajectory")
        except ValueError:
            print("Invalid number of loops or frequency")
        except FileNotFoundError:
            print("File not found")
        finally:
            # Return to the smooth submode for the stand/return-to-ready move that
            # follows a completed gait.
            self._selqie.set_all_motors_position_mode('pos_spd')
            
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
                rate = self._selqie.create_rate(frequency)
                trajectories = self._selqie.get_leg_trajectories_from_file(file, frequency)
                print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
                for i in range(num_loops):
                    print(f"  Loop {i+1}/{num_loops}")
                    self._selqie.run_leg_trajectories(trajectories)
                    rate.sleep()
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
        # Standing is a held pose -> smooth position-speed submode.
        self._selqie.set_all_motors_position_mode('pos_spd')
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
