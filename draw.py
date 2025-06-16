import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

def euler_from_quaternion(self, msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class SignWriter(Node):
    def __init__(self):
        super().__init__('sign_writer')

        # ========== 訂閱與發佈 ========== #
        # 訂閱標誌指令
        self.subscription = self.create_subscription(
            String,
            '/sign_detections',
            self.detection_callback,
            10
        )

        # 訂閱 odom 用於回正方向
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 控制速度
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.last_command = None  

        # ========== 變數初始化 ========== #
        self.current_theta = 0.0

        self.init_theta = None  # 初始朝向（當作正北）
        self.is_writing = False  # 避免重複寫字
        self.pending_command = None  # 等待處理的指令

        # 進場初始化面向
        self.get_logger().info("等待接收第一筆指令...")
        self.create_timer(2.0, lambda: self.get_logger().info("🌀 still alive"))


    def euler_from_quaternion(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    # ========== 方向監聽 ========== #
    def odom_callback(self, msg):
        # 從四元數取得朝向
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(q)
        self.current_theta = yaw
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y

        # 紀錄初始方向
        if self.init_theta is None:
            self.init_theta = yaw
            self.get_logger().info(f"記錄初始朝向 theta: {math.degrees(self.init_theta):.2f} 度")

    # ========== 指令處理 ========== #
    def detection_callback(self, msg):
        command = msg.data.upper()
        
        if command == "STOP":
            self.stop_robot()
            self.is_writing = False

        if command == self.last_command:
            self.get_logger().info(f"🛑 重複指令 {command}，忽略")
            return

        if self.is_writing:
            self.get_logger().warn("正在寫字中，忽略新指令")
            return
        
        self.get_logger().info(f"接收到指令：{command}")
        self.last_command = command
        self.is_writing = True
        
        # 根據指令呼叫對應動作函數
        if command == "W":
            self.is_writing = True
            self.draw_W()
        elif command == "E":
            self.is_writing = True
            self.draw_e()
        elif command == "LOVE":
            self.is_writing = True
            self.draw_heart()
        elif command == "U":
            self.is_writing = True
            self.draw_U()
        
        

    # ========== 動作函數（之後會補） ========== #
    def draw_W(self):
        self.get_logger().info("開始畫 W")
        # self.start_pos_x = self.current_pos_x
        # self.start_pos_y = self.current_pos_y
        # self.start_theta = self.current_theta
        # self.get_logger().info(f"目標位置：({self.start_pos_x:.2f}, {self.start_pos_y:.2f})")
        # 筆劃 1: 右轉 30 度，前進
        self.rotate_in_place(-math.radians(60), 1.0)
        self.forward(0.2, 4.0)

        # 筆劃 2: 左轉 60 度，前進
        self.rotate_in_place(math.radians(130), 1.0)
        self.forward(0.2, 4.0)

        # 筆劃 3: 右轉 60 度，前進
        self.rotate_in_place(-math.radians(130), 1.0)
        self.forward(0.2, 4.0)

        # 筆劃 4: 左轉 30 度，前進
        self.rotate_in_place(math.radians(130), 1.0)
        self.forward(0.2, 4.0)

        self.get_logger().info("W 完成")
        self.get_logger().info("畫完 W，準備回正朝向")    

        # 回正方向
        self.rotate_in_place(-math.radians(70), 1.0)
        # 寫完
        self.is_writing = False      
        self.stop_robot()

    def draw_e(self):
        self.get_logger().info("✏️ 開始畫 e")

        # 1. 畫橫線（往右）
        self.forward(0.2, 2.0)
        self.rotate_in_place(math.radians(90), 1.0)

        # 1.5

        # 2. 畫圓弧（逆時針）
        
        self.draw_arc(linear_speed=0.2, angular_speed=0.5, duration=math.pi / 0.5)

        self.forward(0.2, 1.0)

        self.draw_arc(linear_speed=0.2, angular_speed=0.5, duration=math.pi / 0.5)

        # 3. 固定回轉角度（先估一個）
        self.get_logger().info("🔁 固定轉 -90° 回正")

        
        self.rotate_in_place(-math.radians(90), 1.0)

        self.get_logger().info("✅ e 完成")
        self.is_writing = False


    def draw_heart(self):
        self.get_logger().info("❤️ 開始畫愛心")

        # 左弧（逆時針半圓）
        self.rotate_in_place(math.radians(30), 1.0)
        self.forward(0.3, 4)
        self.rotate_in_place(math.radians(60), 1.0)
        self.draw_arc(linear_speed=0.22, angular_speed=0.5, duration=math.pi / 0.5)  # 半圓約 6.28s

        # 轉向右弧
        self.rotate_in_place(-math.radians(90), 2.0)

        # 右弧（順時針半圓）
        self.draw_arc(linear_speed=0.22, angular_speed=0.5, duration=math.pi / 0.5 + 0.1)  # 負號表示順時針

        # 向下收尾
        self.rotate_in_place(math.radians(30), 1.0)
        self.forward(0.3, 4)  # 長度你可以微調

        # 回正（整體偏轉了 180°）
        self.get_logger().info("🔁 固定轉 -180° 回正")
        self.rotate_in_place(math.radians(60), 1.0)

        self.get_logger().info("✅ 愛心完成")
        self.is_writing = False



    def draw_U(self):
        #U
        # self.rotate_in_place(-math.radians(90), 1.0)
        # self.forward(0.3, 2.5)
        # self.draw_arc(linear_speed=0.15, angular_speed=0.5, duration=math.pi / 0.5)
        # self.forward(0.3, 2.5)

        #J
        self.rotate_in_place(-math.radians(90), 1.0)
        self.draw_arc(linear_speed=0.3, angular_speed=0.5, duration=math.pi / 0.5 )
        self.forward(0.3, 2.5)
        time.sleep(0.5)


        #a
        self.rotate_in_place(math.radians(180), 1.0)
        self.forward(0.3, 1)
        self.draw_arc(linear_speed=0.2, angular_speed=-0.5, duration=2*math.pi / 0.5 )
        self.forward(0.3, 2)
        self.rotate_in_place(math.radians(90), 1.0)
        time.sleep(0.5)

        #e
        self.forward(0.3, 2.0)
        self.rotate_in_place(math.radians(90), 1.0)
        self.draw_arc(linear_speed=0.2, angular_speed=0.5, duration=math.pi / 0.5)
        self.forward(0.2, 1.0)
        self.draw_arc(linear_speed=0.2, angular_speed=0.5, duration=math.pi / 0.5)
        self.rotate_in_place(-math.radians(90), 1.0)
        time.sleep(0.5)

        #s
        self.draw_arc(linear_speed=0.2, angular_speed=0.5, duration=math.pi / 0.5)
        self.draw_arc(linear_speed=0.2, angular_speed=-0.5, duration=math.pi / 0.5)
        time.sleep(0.5)
        
        #i
        self.rotate_in_place(-math.radians(70), 1.0)
        self.forward(0.2, 4.0)
        time.sleep(0.5)

        #k
        # self.rotate_in_place(math.radians(180), 1.0)
        self.forward(0.2, 4.0)
        self.rotate_in_place(-math.radians(180), 1.0)
        self.forward(0.2, 1.5)
        self.rotate_in_place(-math.radians(45), 1.0)
        self.forward(0.2, 2.5)
        self.rotate_in_place(math.radians(180), 1.0)
        self.forward(0.2, 2.5)
        self.rotate_in_place(math.radians(90), 1.0)
        self.forward(0.2, 2.5)
        self.rotate_in_place(math.radians(45), 1.0)
        self.is_writing = False

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("機器人已停止")

# ===================basemove==================
    def forward(self, speed=0.2, duration=1.0):
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)
        self.stop_robot()

    def rotate_in_place(self, angular_speed_rad, duration):
        twist = Twist()
        twist.angular.z = angular_speed_rad

        start_time = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)  # 20Hz 比較穩定S

        self.stop_robot()

    def draw_arc(self, linear_speed, angular_speed, duration):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        start_time = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)   # 可以更頻繁發送
        self.stop_robot()


    # def draw_arc(self, linear_speed=0.2, angular_speed=0.5, duration=1.57):SS
    #     twist = Twist()
    #     twist.linear.x = linear_speed
    #     twist.angular.z = angular_speed
    #     self.cmd_vel_pub.publish(twist)
    #     rclpy.spin_once(self, timeout_sec=duration)
    #     self.stop_robot()


    

def main(args=None):
    rclpy.init(args=args)
    node = SignWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("鍵盤中斷，節點關閉。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    # def correct_orientation(self):
    #     if self.init_theta is None:
    #         return

    #     delta_theta = self.init_theta - self.current_theta
    #     # 角度正規化到 [-pi, pi]
    #     delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

    #     self.get_logger().info(
    #         f"回正方向：目前角度 {math.degrees(self.current_theta):.2f}° → 初始角度 {math.degrees(self.init_theta):.2f}°，差值 {math.degrees(delta_theta):.2f}°"
    #     )

    #     # 如果差距很小，不動
    #     if abs(delta_theta) < math.radians(3):
    #         self.get_logger().info("角度差距小於 3 度，略過回正")
    #         return

    #     # 旋轉修正
    #     angular_speed = 0.5  # 轉速（rad/s）
    #     duration = abs(delta_theta) / angular_speed
    #     self.rotate_in_place(delta_theta, duration)



    # ========== 朝向校正函數 ========== #
    # def correct_pose_to(self, target_x, target_y, target_theta):
    #     self.get_logger().info("🔁 開始回到指定點與朝向...")

    #     dx = target_x - self.current_pos_x
    #     dy = target_y - self.current_pos_y
    #     distance = math.sqrt(dx ** 2 + dy ** 2)
    #     self.get_logger().info(f"目前位置：({self.current_pos_x:.2f}, {self.current_pos_y:.2f})")
    #     self.get_logger().info(f"目標位置：({target_x:.2f}, {target_y:.2f})")
    #     self.get_logger().info(f"距離差距為：{distance:.4f} m")

        

    #     if distance > 0.05:
    #         # 轉向目標點
    #         self.get_logger().info("轉向目標點")
    #         angle_to_target = math.atan2(dy, dx)
    #         delta_theta = angle_to_target - self.current_theta
    #         delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
    #         self.rotate_in_place(delta_theta, abs(delta_theta) / 0.5)

    #         # 前進到目標位置
    #         duration = distance / 0.2
    #         self.forward(speed=0.2, duration=duration)

    #     # 最後轉回起始朝向
    #     delta_theta = target_theta - self.current_theta
    #     delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
    #     if abs(delta_theta) > math.radians(3):
    #         self.rotate_in_place(delta_theta, abs(delta_theta) / 0.5)

    #     self.get_logger().info("✅ 回到起始點並校正方向完畢")


    # ========== 四元數轉歐拉角 ========== #