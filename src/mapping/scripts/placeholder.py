        # Subscribers
        self.drone_pose_sub = rospy.Subscriber('drone1/mavros/local_position/pose', PoseStamped, 
                                               self.drone_pose_callback)
        self.robot_odom_sub = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, 
                                               self.robot_odom_callback)
        self.teleop_sub = rospy.Subscriber('drone1/desired/cmd_vel', Twist, self.teleop_callback) #kig teleop og put det igennem her
        
        # Timer for control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.cmd_callback)
        
        # Publisher for filtered commands
        self.safe_cmd_pub = rospy.Publisher('drone1/incoming/cmd_vel', Twist, queue_size=10) #output safe values to offboard node.