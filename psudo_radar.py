import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PsudoRadar(Node):

    def __init__(self, data_dir):
        super().__init__('psudoRadar_node')
        self.data_dir = data_dir # data_dir : 모든 파일들의 경로
        self.data_index = 0  # 현재 전송할 데이터 파일의 인덱스
        self.publisher_ = self.create_publisher(Float32MultiArray, 'radar_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)  # 메서드가 호출되는 주기를 제어

    def data_simple_loader(self, path):
        try:
            data = np.load(path) # path : 현재 처리 중인 파일 하나의 경로
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to load data from {path}: {e}")
            return None

    def publish_data(self):
        if self.data_index >= len(self.data_dir):
            self.get_logger().info("All data has been published.")
            rclpy.shutdown()
            return

        current_path = self.data_dir[self.data_index]
        data = self.data_simple_loader(current_path)

        if data is not None:
            msg = Float32MultiArray()
            msg.data = data.flatten().tolist()  # 데이터를 리스트로 변환하여 메시지에 설정
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published radar data from: {current_path} with shape {data.shape}')
        
        self.data_index += 1  # 다음 데이터 파일로 이동


def main(args=None):

    rclpy.init(args=args)

    data_dir = os.listdir('/home/zeta/Can_do/rtnh_wider_1p_1/15')
    data_dir = np.sort(data_dir)
    data_dir = [f"/home/zeta/Can_do/rtnh_wider_1p_1/15/{filename}" for filename in data_dir]

    psudo_radar_node = PsudoRadar(data_dir)

    try:
        rclpy.spin(psudo_radar_node)
    except Exception as e:
        psudo_radar_node.get_logger().error(f"An error occurred: {e}")
    finally:
        psudo_radar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''''
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import os


class PsudoRadar(Node):

    def __init__(self, data_list):
        super().__init__('psudoRadar_node') 
        self.data_list = data_list #  file list 
        self.index = 0 # file index 
        self.publisher_ = self.create_publisher(Float32MultiArray, 'radar_data',10 )
        self.timer = self.create_timer(4.0, self.publish_data) # 메서드가 호출되는 주기를 제어


    def data_simple_loader(self,path): #current file path

        try:
            data = np.load(path)
            return data
        
        except Exception as e:
            self.get_logger().error(f"Failed to load data from {path}: {e}")
            return None


    def publish_data(self):    

        if self.index >= len(self.data_list):
            self.get_logger().info("All data has been published.")
            rclpy.shutdown()
            return
        
        current_path = self.data_list[self.index]
        data = self.data_simple_loader(current_path) 
        
        if data is not None:
            msg = Float32MultiArray()
            msg.data = data.flatten().tolist()  # 각 data_point를 리스트로 변환하여 설정
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published radar data point : {data.shape}')
        
        self.index += 1  # 다음 데이터 파일로 이동
                   
  

def main(args=None):

    rclpy.init(args=args) 
    
    data_list = os.listdir('/home/zeta/Can_do/test_data/')
    data_list = np.sort(data_list)
    data_list  = [f"/home/zeta/Can_do/test_data/{filename}" for filename in data_list]
    
    psudo_radar_node = PsudoRadar(data_list)

    try:
        # rclpy.spin(psudo_radar_node)
        rclpy.spin_once(psudo_radar_node) 

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        psudo_radar_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
        main()

'''
'''''''''
    # def publish_data(self):

        data = self.data_simple_loader() 

        if data is not None:
            msg = Float32MultiArray()
            msg.data = data.flatten().tolist() # 1d array -> list
            
            
            # dimension information
            # dim_0_size = data.shape[0] # 150000
            # dim_1_size = data.shape[1] # 4

            # stride_1 = dim_1_size # 4
            # stride_0 = dim_0_size * stride_1 #600000

            # msg.layout.dim = [
            #     MultiArrayDimension(label='dim_0', size=dim_0_size, stride=stride_0),
            #     MultiArrayDimension(label='dim_1', size=dim_1_size, stride=stride_1)
            # ]
            

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published radar data with shape: {data.shape}')


    def publish_data(self):
         
        data = self.data_simple_loader() 

        if data is not None:
            for i, data_point in enumerate(data):
                msg = Float32MultiArray()
                msg.data = data_point.flatten().tolist()  # 각 data_point를 1차원 리스트로 변환하여 설정
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published radar data point {i+1}/{len(data)} with shape: {data_point.shape}')

        ##############################
        try:
            while rclpy.ok():  # ROS2 시스템이 활성화된 동안 계속 실행
                data = self.data_simple_loader() 

                if data is not None:
                    for i, data_point in enumerate(data):
                        msg = Float32MultiArray()
                        msg.data = data_point.flatten().tolist()  # 각 data_point를 1차원 리스트로 변환하여 설정
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Published radar data point {i+1}/{len(data)} with shape: {data_point.shape}')
                        
                        # 필요한 경우, 각 데이터 포인트 퍼블리시 후 대기 시간 추가
                        time.sleep()  # 데이터를 퍼블리시한 후 짧은 대기 (0.1초 예시)

                # 전체 데이터 퍼블리시 후 잠시 대기, 필요시 제거 가능
                # time.sleep(5.0)  # 전체 데이터를 퍼블리시한 후 1초 대기

        except KeyboardInterrupt:
            self.get_logger().info('Publishing loop interrupted by user.')
    
        finally:
            self.get_logger().info('Shutting down node.')
         ##############################
        



    def main(args=None):

        rclpy.init(args=args) 

        try:
                path = f"/home/zeta/Can_do/test_data/sprdr_00038.npy"
                psudo_radar_node = PsudoRadar(path)
                rclpy.spin(psudo_radar_node)
                # time.sleep(0.01)
        
        except Exception as e:
            print(f"ERROR: {e}")

        finally:
                psudo_radar_node.destroy_node()
                rclpy.shutdown()
           
    '''''''''
'''''''''
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time


class PsudoRadar(Node):

    def __init__(self, path):
        super().__init__('psudo_radar')
        self.path = path
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'radar_data', # Topic name
            10 # Queue Size
        )
        self.timer = self.create_timer(1.0, self.publish_data)

    def data_simple_loader(self):
        try:
            data = np.load(self.path)
            print("data_shape:", data.shape)       
            return data
        except Exception as e:
            print(f"ERROR: {e}")
            return None

    def publish_data(self):
        data = self.data_simple_loader()
        if data is not None:
            msg = Float32MultiArray()
            msg.data = data.flatten().tolist()

            # 필요한 부분인지 재확인 필요!

            dim_0_size = data.shape[0]
            dim_1_size = data.shape[1]

            stride_1 = dim_1_size
            stride_0 = dim_0_size * stride_1

            msg.layout.dim = [
                MultiArrayDimension(label='dim_0', size=dim_0_size, stride=stride_0),
                MultiArrayDimension(label='dim_1', size=dim_1_size, stride=stride_1)
            ]

        # 데이터를 1차원 리스트로 변환하여 퍼블리시
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published radar data with shape: {data.shape}')


def main(args=None):

    rclpy.init(args=args)
    
    indice = [38,39]
    for i in indice :
        path = f"/home/zeta/Can_do/test_data/sprdr_000{i}.npy"
        print(path)

        try:
            psudo_radar_node = PsudoRadar(path)
            rclpy.spin_once(psudo_radar_node, timeout_sec=1.0)


        except Exception as e:
            print(f"ERROR: {e}")
            return None
        
        finally:
            time.sleep(0.5)
            print("pass")

    psudo_radar_node.destroy_node()
    rclpy.shutdown()

    # each point publish 
    def main(args=None):

        rclpy.init(args=args) 

        try:
            path = f"/home/zeta/Can_do/test_data/sprdr_00038.npy"
            psudo_radar_node = PsudoRadar(path)
            # rclpy.spin(psudo_radar_node)
            rclpy.spin_once(psudo_radar_node) 

        except Exception as e:
            print(f"ERROR: {e}")

        finally:
                psudo_radar_node.destroy_node()
                rclpy.shutdown()   


if __name__ == '__main__':
    main()
'''''''''