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
        
        # 订阅迁移包 (Best Effort 可能会丢包导致曲线断裂，建议 Reliable)
        # 这里保持你原脚本的默认 QoS (10)
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
        
        self.get_logger().info('📉 Generation Monitor Started. Waiting for packets...')

    def listener_callback(self, msg):
        island_id = msg.source_id
        
        # 获取该包里最好的个体
        if not msg.individuals:
            return
            
        best_fitness = msg.individuals[0].fitness
        
        # 🔥 核心修改点 1：直接从消息中获取代数 (Generation)
        generation = msg.generation 
        
        with self.lock:
            if island_id not in self.data:
                self.data[island_id] = {'x': [], 'y': []}
                
            # 简单去重：防止同一代发了多次包导致图表回退
            # 只有当新收到的代数大于当前记录的最大代数时才添加
            if not self.data[island_id]['x'] or generation > self.data[island_id]['x'][-1]:
                self.data[island_id]['x'].append(generation) # 🔥 X轴存代数
                self.data[island_id]['y'].append(best_fitness)

    def update_plot(self, frame):
        with self.lock:
            self.ax.clear()
            self.ax.set_title("Real-time Evolution Progress (By Generation)")
            self.ax.set_xlabel("Generations") # 🔥 核心修改点 2：标签改为代数
            self.ax.set_ylabel("Best Fitness (Log Scale)")
            self.ax.set_yscale('log') # 保持对数坐标
            self.ax.grid(True, which="both", ls="-", alpha=0.5)
            
            # 使用 Tab20 颜色映射，因为默认颜色循环只有10种，20个岛不够分
            cmap = plt.get_cmap('tab20')
            
            # 对 ID 排序，保证图例顺序固定
            # 假设 ID 格式为 "Island_0", "Island_1" 等
            sorted_ids = sorted(self.data.keys(), key=lambda x: int(x.split('_')[-1]) if '_' in x else x)

            for i, island_id in enumerate(sorted_ids):
                packet = self.data[island_id]
                if len(packet['x']) > 0:
                    # 获取当前最新 Fitness 用于图例显示
                    current_val = packet['y'][-1]
                    label_str = f"{island_id} ({current_val:.2e})"
                    
                    self.ax.plot(packet['x'], packet['y'], 
                                 label=label_str,
                                 color=cmap(i % 20), # 自动分配颜色
                                 linewidth=1.5, alpha=0.8)
            
            # 图例放在右上角，字体稍微小一点防止遮挡
            self.ax.legend(loc='upper right', fontsize='small', ncol=1)

def main(args=None):
    rclpy.init(args=args)
    monitor_node = EvoMonitor()
    
    # ROS Spin 线程
    thread = threading.Thread(target=rclpy.spin, args=(monitor_node,), daemon=True)
    thread.start()
    
    # 动画主线程
    ani = FuncAnimation(monitor_node.fig, monitor_node.update_plot, interval=500) # 500ms 刷新一次
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
