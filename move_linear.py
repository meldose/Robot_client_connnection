
def movelinear_online(self,*args,**kwargs):
    
    """
    Method to do Servo in Cartesian Space
    
    Args:
    - servo_target_position ([int]*7)     : List of cartesian target position in m. Length of list should be equal to 7 (X, Y, Z, qw, qx, qz, qz)
    - servo_target_velocity ([int]*7)     : List of cartesian target velocity in m/s. Length of list should be equal to 7
    - servo_target_acceleration ([int]*7) : List of cartesian target acceleration in m/s^2. Length of list should be equal 7. Not used in Current Version
    - gain parameter (double)             : ServoX gain parameter, used for Propotional controller (should be between [0.2, 100])

    Returns:
    - Servo warning and error codes

    **Sample Usage**
    
        .. code-block:: python

            from neurapy.robot import Robot
            import time
            from ruckig import InputParameter, OutputParameter, Result, Ruckig
            import copy

            r = Robot()

            #Switch to external servo mode
            r.activate_servo_interface('position')

            cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz

            otg = Ruckig(cart_pose_length, 0.001)  # control cycle
            inp = InputParameter(cart_pose_length)
            out = OutputParameter(cart_pose_length)

            inp.current_position = r.get_current_cartesian_pose()
            inp.current_velocity = [0.]*cart_pose_length
            inp.current_acceleration = [0.]*cart_pose_length

            target = copy.deepcopy(inp.current_position)
            target[0] += 0.2 # Move 200mm in X direction
            inp.target_position = target
            inp.target_velocity = [0.]*cart_pose_length
            inp.target_acceleration = [0.]*cart_pose_length

            inp.max_velocity = [0.5]*cart_pose_length
            inp.max_acceleration = [3]*cart_pose_length
            inp.max_jerk = [10.]*cart_pose_length
            res = Result.Working

            servox_proportional_gain = 25

            while res == Result.Working:
                '''
                Error code is returned through Servo. 
                '''
                error_code = 0
                if(error_code < 3):

                    res = otg.update(inp, out)

                    position = out.new_position
                    velocity = out.new_velocity 
                    acceleration = out.new_acceleration

                    error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
                    scaling_factor = r.get_servo_trajectory_scaling_factor()
                    out.pass_to_input(inp)
                    time.sleep(0.001)
                else:
                    print("Servo in error, error code, ", error_code)
                    break
            r.deactivate_servo_interface()

            r.stop()
    """
    
    self.logger.info(
            "MOVELINEAR called with parameters {} {}".format(args, kwargs)
        )
    if("maira" in self.robot_name.lower()):
        self.robot.logger.warning("MOVELINEAR not implemented for Maira. Stay tuned for updates.")
        return False
    
    command = Servo(self)
    command.execute_visual_servoing(*args,**kwargs)

