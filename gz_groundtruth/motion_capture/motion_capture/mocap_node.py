import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseArray, 
    Pose,
    Point,
    Quaternion)

class MocapNode (Node):
    def __init__(self) -> None:
        super().__init__('mocap_node')
        # Creazione del subscriber
        self.subscriber_ = self.create_subscription(
            PoseArray, '/world/default/pose/info', self.listener_cb, 10)

    def listener_cb(self, msg: PoseArray):
        # Nell'array di oggetti Pose il secondo è quello del drone a cui siamo interessati. L'ho
        # trovato a tentativi guardando i numeri più convincenti e sono sicuro sia quello perchè 
        # coincide con la visuale nella simulazione. E' ripetibile tutte le simulazioni e funziona sempre
        # ma penso valga solo per la specifica simulazione 'make px4_sitl gz_x500'. 
        pose : Pose = msg.poses[1]

        # Parsing e output dei dati
        position : Point = pose.position
        orientation : Quaternion = pose.orientation
        output = f"---\nPosition:\nx={position.x}\ny={position.y}\nz={position.z}\n"
        output += f"Orientation:\n1={orientation.x}\ni={orientation.y}\nj={orientation.z}\nk={orientation.w}\n"
        self.get_logger().info(output)

def main(args=None):
    rclpy.init(args=args)
    sub = MocapNode()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
