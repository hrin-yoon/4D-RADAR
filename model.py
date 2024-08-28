import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class NNmodel(Node):

    def __init__(self):
        super().__init__('nnmodel')

        # 'processed_data' 토픽을 구독하는 구독자 생성
        self.subscription_nn = self.create_subscription( Float32MultiArray,'processed_data', self.sub_feeder_msg,10)
        # 퍼블리셔 추가
        self.publisher_infer = self.create_publisher(Float32MultiArray, 'nnmodel_data', 10)


    def sub_feeder_msg(self, msg):
        # 수신된 메시지를 처리하는 콜백 함수
        data = msg.data
        self.get_logger().info(f"Subscribing processed data: {len(data)}")

        # 퍼블리시를 위해 수신된 데이터 처리
        self.pub_nnmodel_msg(msg)

    def pub_nnmodel_msg(self,msg):
        # 수신된 데이터를 다른 토픽으로 퍼블리시하는 함수
        # 데이터 처리 코드 넣을 예정
        data = msg.data
        self.publisher_infer.publish(msg)
        self.get_logger().info(f"Publishing processed data: {len(msg.data)}")

def main(args=None):
    rclpy.init(args=args)
    nn_model = NNmodel()
    
    try:
        rclpy.spin(nn_model)

    except Exception as e:
        print(f"ERROR: {e}")
        return None
    
    finally:
        nn_model.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
