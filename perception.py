import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

class Perception(Node):
    def __init__(self):
        super().__init__('perception')

        self.subscription_percep = self.create_subscription(Float32MultiArray, 'radar_data', self.sub_nnmodel_msg, 10)
        self.publisher_percep = self.create_publisher(PointCloud2, 'visual_rviz', 10)

    def sub_nnmodel_msg(self, msg):

        data = msg.data
        self.get_logger().info(f"Subscribing processed data: {len(data)}")

        num_sets = len(data) // (150000 * 4)
        for i in range(num_sets):
            start_idx = i * 150000 * 4
            end_idx = (i + 1) * 150000 * 4
            reshaped_array = np.array(data[start_idx:end_idx]).reshape(150000, 4)
            self.pub_vis_msg(reshaped_array)
        
    def pub_vis_msg(self, reshaped_array): 

        msg = PointCloud2()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # 적절한 frame_id를 사용하세요.

        msg.height = 1
        msg.width = reshaped_array.shape[0]

        # PointCloud2의 필드 정의
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 4 (x) + 4 (y) + 4 (z) + 4 (intensity)
        msg.row_step = msg.point_step * reshaped_array.shape[0]

        msg.is_dense = True
        msg.data = np.array(reshaped_array, dtype=np.float32).tobytes()

        # PointCloud2 메시지 발행
        self.publisher_percep.publish(msg)
        self.get_logger().info(f"Published PointCloud2 with {reshaped_array.shape[0]} points")

def main(args=None):

    rclpy.init(args=args)

    percep = Perception()
    
    try:
        rclpy.spin(percep)
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        percep.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class Perception(Node):
    def __init__(self):
        super().__init__('perception')

        # 'nnmodel' 토픽을 구독하는 구독자 생성
        self.subscription_percep = self.create_subscription( Float32MultiArray,'radar_data', self.sub_nnmodel_msg,10)
        self.publisher_percep = self.create_publisher( Float32MultiArray, 'visual_rviz', 10)


    def sub_nnmodel_msg(self, msg):
        # 수신된 메시지를 처리하는 콜백 함수
        data = msg.data
        self.get_logger().info(f"Subscribing processed data: {len(data)}")
        self.pub_vis_msg(msg)
        
        # reshaped_array = np.array(data).reshape(150000, 4)
        # self.get_logger().info(f"reshaped_array: {reshaped_array.shape}")

    def pub_vis_msg(self, msg): 

        data = msg.data
        reshaped_array = np.array(data).reshape(150000, 4)
        self.publisher_percep.publish(reshaped_array)
        self.get_logger().info(f"reshaped_array: {reshaped_array.shape}")

    

def main(args=None):
    rclpy.init(args=args)
    percep = Perception()
    
    try:
        rclpy.spin(percep)

    except Exception as e:
        print(f"ERROR: {e}")
        return None
    
    finally:
        percep.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

 ## ver 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np


class Perception(Node):
    def __init__(self):
        super().__init__('perception')

      
        self.subscription_percep = self.create_subscription(Float32MultiArray, 'radar_data', self.sub_nnmodel_msg, 10)

        self.publisher_percep = self.create_publisher(PointCloud2, 'visual_rviz', 10)

    def sub_nnmodel_msg(self, msg):
     
        data = msg.data
        self.get_logger().info(f"Subscribing processed data: {len(data)}")

        reshaped_array = np.array(data).reshape(150000, 4)
        self.pub_vis_msg(reshaped_array)
        
    def pub_vis_msg(self, reshaped_array): 

        msg = PointCloud2()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  

        msg.height = 1
        msg.width = reshaped_array.shape[0]


        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 4 (x) + 4 (y) + 4 (z) + 4 (intensity)
        msg.row_step = msg.point_step * reshaped_array.shape[0]

        msg.is_dense = True
        msg.data = np.array(reshaped_array, dtype=np.float32).tobytes()

        # PointCloud2 메시지 발행
        self.publisher_percep.publish(msg)
        self.get_logger().info(f"Published PointCloud2 with {reshaped_array.shape[0]} points")

def main(args=None):
    rclpy.init(args=args)
    percep = Perception()
    
    try:
        rclpy.spin(percep)
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        percep.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''