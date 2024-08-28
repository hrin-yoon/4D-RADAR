import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DataFeeder(Node):
    cont = 0
    def __init__(self):
        super().__init__('data_feeder')
        
        # 'radar_data' 토픽을 구독하는 구독자 생성
        self.subscription_radar = self.create_subscription( Float32MultiArray,'radar_data', self.sub_radar_msg, 10)
        # 퍼블리셔 추가
        self.publisher_feeder = self.create_publisher(Float32MultiArray, 'processed_data', 10)

    def sub_radar_msg(self, msg): 
        # 수신된 메시지를 처리하는 콜백 함수
        data = msg.data
        self.get_logger().info(f"Subscribing processed data: {len(data)}")

        # 퍼블리시를 위해 수신된 데이터 처리
        # ROS2에서 메시지를 sub 하고 그 데이터를 처리하여 다른 토픽에 pub
        self.pub_model_msg(msg)

    def pub_model_msg(self, msg): 
        # 수신된 데이터를 다른 토픽으로 퍼블리시하는 함수
        # 데이터 처리 코드 넣을 예정
        data = msg.data
        self.publisher_feeder.publish(msg)
        self.get_logger().info(f"Publishing processed data: {len(msg.data)}")
        # get_logger : ROS 2 토픽에서 메시지를 수신했을 때 호출되는 함수


def main(args=None):

    rclpy.init(args=args) # pub, sub 커뮤니케이션 초기화
    data_feeder = DataFeeder() 
    
    try:
        rclpy.spin(data_feeder) # 콜백 실행

    except Exception as e:
        print(f"ERROR: {e}")
        return None
    
    finally:
        data_feeder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
