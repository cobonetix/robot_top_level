import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import ArmJoint


class JointServiceClient(Node):
    def __init__(self):
        super().__init__('joint_service_client')
        self.client = self.create_client(ArmJoint, 'arm_joint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm_joint service...')
        self.get_logger().info('Connected to arm_joint service')

    def send_request(self, l_j1, l_j2, l_j3, l_j4, r_j1, r_j2, r_j3, r_j4):
        request = ArmJoint.Request()
        request.l_j1 = l_j1
        request.l_j2 = l_j2
        request.l_j3 = l_j3
        request.l_j4 = l_j4
        request.r_j1 = r_j1
        request.r_j2 = r_j2
        request.r_j3 = r_j3
        request.r_j4 = r_j4
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    client = JointServiceClient()

    while True:
        try:
            user_input = input("Enter l_j1 l_j2 l_j3 l_j4 r_j1 r_j2 r_j3 r_j4: ").split()
            if len(user_input) != 8:
                print("Invalid input. Please enter 8 values: l_j1 l_j2 l_j3 l_j4 r_j1 r_j2 r_j3 r_j4")
                continue

            l_j1 = float(user_input[0])
            l_j2 = float(user_input[1])
            l_j3 = float(user_input[2])
            l_j4 = float(user_input[3])
            r_j1 = float(user_input[4])
            r_j2 = float(user_input[5])
            r_j3 = float(user_input[6])
            r_j4 = float(user_input[7])

            print(f"Sending request: left=[{l_j1}, {l_j2}, {l_j3}, {l_j4}], right=[{r_j1}, {r_j2}, {r_j3}, {r_j4}]")

            future = client.send_request(l_j1, l_j2, l_j3, l_j4, r_j1, r_j2, r_j3, r_j4)
            rclpy.spin_until_future_complete(client, future)

            result = future.result()
            print(f"Response: success={result.success}, message={result.message}")

        except ValueError:
            print("Invalid input. Please enter numeric values for all joints.")
        except KeyboardInterrupt:
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
