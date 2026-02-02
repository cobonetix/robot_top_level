import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import ArmPose


class PoseServiceClient(Node):
    def __init__(self):
        super().__init__('pose_service_client')
        self.client = self.create_client(ArmPose, 'arm_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm_pose service...')
        self.get_logger().info('Connected to arm_pose service')

    def send_request(self, arm, x, y, z, rotation):
        request = ArmPose.Request()
        request.arm = arm
        request.x = x
        request.y = y
        request.z = z
        request.rotation = rotation
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    client = PoseServiceClient()

    while True:
        try:
            user_input = input("Enter arm,x,y,z,rotation (e.g., l 1.0 2.0 3.0 0.5): ").split()
            if len(user_input) != 5:
                print("Invalid input. Please enter 5 values: arm x y z rotation")
                continue

            arm = user_input[0]
            x = float(user_input[1])
            y = float(user_input[2])
            z = float(user_input[3])
            rotation = float(user_input[4])

            print(f"Sending request: arm={arm}, x={x}, y={y}, z={z}, rotation={rotation}")

            future = client.send_request(arm, x, y, z, rotation)
            rclpy.spin_until_future_complete(client, future)

            result = future.result()
            print(f"Response: success={result.success}, message={result.message}")

        except ValueError:
            print("Invalid input. Please enter numeric values for x, y, z, rotation.")
        except KeyboardInterrupt:
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
