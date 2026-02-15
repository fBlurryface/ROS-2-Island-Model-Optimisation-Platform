#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from island_evo_core.msg import Packet
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class EvoMonitor(Node):
    def __init__(self):
        super().__init__('evo_monitor')
        
        # 订阅迁移包
        self.subscription = self.create_subscription(
            Packet,
            'migration_packets',
            self.listener_callback,
            10)
        
        # 数据存储: { 'Island_0': {'gen': [], 'fit': []}, ... }
        self.data = {}
        self.lock = threading.Lock()
        
        # 绘图初始化
        self.fig, self.ax = plt.subplots()
        self.lines = {}
        
        self.get_logger().info('📉 Evolution Monitor Started. Waiting for data...')

    def listener_callback(self, msg):
        island_id = msg.source_id
        
        # 获取该包里最好的个体
        if not msg.individuals:
            return
            
        # 假设包里的第一个就是最好的（通常发送逻辑如此）
        best_fitness = msg.individuals[0].fitness
        
        # 这里我们需要估算代数。
        # 由于 Packet 定义里没有 explicit generation 字段 (之前的 msg 定义)，
        # 我们通常只能按收到的顺序画，或者如果在 msg 里加了 gen 字段更好。
        # ⚠️ 临时方案：用 len(self.data[island_id]['fit']) 作为 x 轴
        # 或者如果你在 Individual msg 里存了 genes，无法反推代数。
        # 建议：仅仅画出“接收到的次数”或者简单的时序
        
        with self.lock:
            if island_id not in self.data:
                self.data[island_id] = {'x': [], 'y': []}
                
            # 记录数据
            # 为了曲线平滑，可以简单的用计数器当 X 轴，或者如果 Packet 包含 gen 更好
            # 这里我们假设数据是按顺序来的
            current_x = len(self.data[island_id]['x']) + 1
            self.data[island_id]['x'].append(current_x)
            self.data[island_id]['y'].append(best_fitness)

    def update_plot(self, frame):
        with self.lock:
            self.ax.clear()
            self.ax.set_title("Real-time Evolution Progress")
            self.ax.set_xlabel("Migration Events")
            self.ax.set_ylabel("Best Fitness (Log Scale)")
            self.ax.set_yscale('log') # 对数坐标看收敛更清晰
            self.ax.grid(True, which="both", ls="-", alpha=0.5)
            
            for island_id, packet in self.data.items():
                # 绘制每个岛屿的曲线
                if len(packet['x']) > 0:
                    self.ax.plot(packet['x'], packet['y'], label=island_id)
            
            self.ax.legend(loc='upper right')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = EvoMonitor()
    
    # 因为 matplotlib 需要在主线程运行，我们把 ros spin 放到子线程
    thread = threading.Thread(target=rclpy.spin, args=(monitor_node,), daemon=True)
    thread.start()
    
    # 启动动画
    ani = FuncAnimation(monitor_node.fig, monitor_node.update_plot, interval=500)
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
