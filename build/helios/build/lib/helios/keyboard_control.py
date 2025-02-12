import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyJointEffort
from pynput import keyboard

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        self.joint_names = [
            "HELIOS::rotor_0_joint", #Front Right
            "HELIOS::rotor_1_joint", #Back Left
            "HELIOS::rotor_2_joint", #Front Left
            "HELIOS::rotor_3_joint"  #Back Right
        ]

        self.effort = 0.0
        self.effort_step=0.5
        self.running=True
        self.key_states={
            "w":False, "s":False, "a":False, "d":False,
            "i":False, "k":False, "j":False, "l":False
        }

        self.client = self.create_client(ApplyJointEffort, "/gazebo/apply_joint_effort")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /gazebo/apply_joint_effort service...")
        
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        self.timer = self.create_timer(0.1, self.update_effort)
    
    def on_press(self,key):
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = True
        except AttributeError:
            pass
    
    def on_release(self, key):
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = False
        except AttributeError:
            pass
    
    def update_effort(self):
        effort_vals = [0.0,0.0,0.0,0.0]

        if self.key_states["w"]:  # Ascend
            effort_vals = [self.effort_step] * 4
        elif self.key_states["s"]:  # Descend
            effort_vals = [-self.effort_step] * 4
        if self.key_states["a"]:  # Roll Left : Right inc, Left dec
            effort_vals = [x + self.effort_step if i % 2 == 0 else x - self.effort_step for i, x in enumerate(effort_vals)]
        if self.key_states["d"]:  # Roll Right : Right dec, Left inc
            effort_vals = [x - self.effort_step if i % 2 == 0 else x + self.effort_step for i, x in enumerate(effort_vals)]
        if self.key_states["i"]:  # Pitch Forward : Front dec, Back inc
            effort_vals = [x - self.effort_step if i == 0 else x + self.effort_step if i == 1 else x for i, x in enumerate(effort_vals)]
        if self.key_states["k"]:  # Pitch Backward : Front inc, Back dec
            effort_vals = [x + self.effort_step if i == 0 else x - self.effort_step if i == 1 else x for i, x in enumerate(effort_vals)]
        if self.key_states["j"]:  # Yaw Left : 01 inc, 23 dec
            effort_vals = [self.effort_step, self.effort_step, -self.effort_step, -self.effort_step]
        if self.key_states["l"]:  # Yaw Right : 01 dec, 23 inc
            effort_vals = [-self.effort_step, -self.effort_step, self.effort_step, self.effort_step]

        for joint, effort in zip(self.joint_names, effort_vals):
            self.apply_effort(joint,effort)
    
    def apply_effort(self, joint_name, effort):
        request = ApplyJointEffort.Request()
        request.joint_name = joint_name
        request.effort = effort
        request.start_time.sec = 0
        request.start_time.nanosec = 0
        request.duration.sec = 1
        request.duration.nanosec = 0
        self.client.call_async(request)
    

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()


"""
Upward movement: Increase the speed of all rotors. 
Downward movement: Decrease the speed of all rotors. 
Forward movement: Increase the speed of the front rotors slightly while decreasing the speed of the rear rotors. 
Backward movement: Reverse the above action, increasing the speed of the rear rotors and decreasing the front. 
Turning (yaw): By increasing the speed of one pair of diagonally opposite rotors while decreasing the speed of the other pair, the drone can rotate around its vertical axis. 
"""