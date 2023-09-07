import rospy
import std_msgs.msg as message


class ROSPublisher():
    """
    Class that publishes data to specified ROS topics, it is made to facilitate the use of the ros publishing.
    After instantiating it, you can use the methods to publish to a topic, in two ways:

    1. publish_once: publishes a message once
    2. publish_recur: publishes a message recurrently

    If you use publish once, initialize this class with the flag signal=True !
    When you finished publishing, using the killnode method!

    If you use the publish_recur method, you need to leave the signal=False and terminate the publisher simply with CTRL + C in the terminal
    """

    def __init__(self, node_name: str, topic: str, data: str, queue: int, anonymous: bool = True, signal: bool = False, verbose: bool = True) -> None:
        """Initialized a ROS Publisher node.

        Args:
            node_name (str): name to be assigned to the node
            topic (str): topic to publish to
            data (str): data type from the std_msgs library, passed as a string (example: "Float32" -> std_msgs.msg.Float32)
            queue (int): queue size for the publisher
            anonymous (bool, optional): If set to true give unique name to same named nodes. Defaults to True.
            signal (bool, optional): If set to true, allow the use of method killnode, to manually kill the rosnode. Defaults to False.
            verbose (bool, optional): If set to true, logs info about the publisher behaviour. Defaults to True.
        """
        self.node_name = node_name
        self.anonymous = anonymous
        self.verbose = verbose
        self.topic = topic
        self.queue = queue
        self.data = getattr(message, data)
        self.signal = signal

        try:
            self.publisher = rospy.Publisher(self.topic, self.data, queue_size=self.queue)
            rospy.init_node(self.node_name, anonymous=self.anonymous, disable_signals=self.signal)

            if self.verbose:
                # log when node starts (really useful for debugging)
                rospy.loginfo(f"Started publisher {self.node_name} to topic: {self.topic}")

        except rospy.ROSException:
            print("failed to initialize publisher!")
            pass

    def publish_once(self, message):
        """Publish a message once to the given topic

        Args:
            message (_type_): message of the same type of the data specified when instatiating the ROS pub object
        """

        try:
            self.publisher.publish(message)  # publish the message
            if self.verbose:
                rospy.loginfo(f"Node {self.node_name} published: {message}")
            return

        except rospy.ROSException:
            print("failed to publish!")
            return

    def publish_recur(self, message, rate_pub: int):
        """Publish a message to the given topic recurrently with a rate_pub frequency (Hz)

        Args:
            message (_type_): message of the same type of the data specified when instatiating the ROS pub object
            rate_pub (int): rate publishing frequency in Hertz
        """
        rate = rospy.Rate(rate_pub)
        try:
            while not rospy.is_shutdown():
                self.publisher.publish(message)  # publish the message
                rate.sleep()  # sleep for the amount setted with Rate

                if self.verbose:
                    rospy.loginfo(
                        f"Node {self.node_name} published: {message}")

        except rospy.ROSInterruptException:
            print("publisher interruped")
            return

    def killnode(self):
        """Kills the node manually. Use this method only if you use the publish_once 
        method and have set the signal=True flag
        when instantiating the ROSPublisher object.
        """
        assert self.signal is not False, "Flag signal was set to False instead of True.\nUnable to kill node!"
        rospy.signal_shutdown("Node publisher was killed manually")
