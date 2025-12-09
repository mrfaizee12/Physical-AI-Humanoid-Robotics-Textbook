---
sidebar_position: 2
sidebar_label: "2. Python Control with rclpy"
---

# 2. Python Control with rclpy

Now that you understand the core concepts of the ROS Graph, let's bring them to life with Python. The official Python client library for ROS 2 is called `rclpy` (ROS Client Library for Python). It provides the tools you need to create nodes, publish and subscribe to topics, and use services.

## Setting Up a Python ROS 2 Node

A ROS 2 node written in Python is fundamentally a script that uses the `rclpy` library. Here is the basic structure of a minimal node:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # 1. Initialize the rclpy library
    rclpy.init(args=args)

    # 2. Create a class that inherits from Node
    my_node = Node('my_first_node')

    # 3. "Spin" the node to keep it running and responsive
    my_node.get_logger().info('My first node is running!')
    rclpy.spin(my_node)

    # 4. Shutdown the rclpy library
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The `rclpy.spin()` function is critical. It enters a loop that processes any incoming events, such as messages arriving on a subscribed topic or service requests. The node will do nothing without it.

## Creating a Publisher

Let's expand on this to create a node that publishes a simple string message to a topic. This is the "Hello, World!" of robotics.

First, we need to import the message type we want to use. For simple text, we can use the `String` message from the `std_msgs` package.

Here is the complete code for a simple publisher node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        
        # Create a publisher on the 'chatter' topic, using the String message type
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create a timer that calls the timer_callback function every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.i = 0

    def timer_callback(self):
        # Create a new String message
        msg = String()
        msg.data = f'Hello World: {self.i}'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run This Code

1.  Save the code above as a Python file (e.g., `simple_publisher.py`).
2.  Make sure your ROS 2 environment is sourced.
3.  Run the node: `python3 simple_publisher.py`
4.  In a **separate terminal** (with ROS 2 sourced), you can listen to the topic using the ROS 2 command-line interface:
    ```bash
    ros2 topic echo /chatter
    ```
You will see the "Hello World" messages printed in the second terminal. This demonstrates a complete publisher-subscriber interaction, where your Python node is the publisher and the `ros2 topic echo` command is the subscriber.
