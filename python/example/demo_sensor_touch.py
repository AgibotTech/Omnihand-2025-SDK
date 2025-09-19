# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO10, EFinger
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec

class TouchSensorVisualizer:
    def __init__(self):
        self.hand = AgibotHandO10()
        self.fig = plt.figure(figsize=(20, 8))
        gs = GridSpec(2, 5, figure=self.fig)
        
        # 设置黑色背景
        self.fig.patch.set_facecolor('black')
        plt.rcParams['text.color'] = 'white'
        plt.rcParams['axes.facecolor'] = 'black'
        
        self.sensors = {
            'Thumb':  (EFinger.THUMB, (4, 4), gs[0, 0]),
            'Index':  (EFinger.INDEX, (4, 4), gs[0, 1]),
            'Middle': (EFinger.MIDDLE, (4, 4), gs[0, 2]),
            'Ring':   (EFinger.RING, (4, 4), gs[0, 3]),
            'Little': (EFinger.LITTLE, (4, 4), gs[0, 4]),
            'Palm':   (EFinger.PALM, (5, 5), gs[1, 1:3]),
            'Dorsum': (EFinger.DORSUM, (5, 5), gs[1, 3:5])
        }

        self.heatmaps = {}
        self.texts = {}
        self._setup_plots()

    def _setup_plots(self):
        for name, (finger, size, pos) in self.sensors.items():
            ax = self.fig.add_subplot(pos)
            ax.set_facecolor('black')
            
            # 创建热力图，使用更鲜明的颜色映射
            heatmap = ax.imshow(np.zeros(size), 
                              cmap='plasma',  # 更鲜明的颜色映射
                              aspect='equal',
                              vmin=0,
                              vmax=255)  # 设置固定的值范围
            
            # 设置标题
            ax.set_title(f'{name}', color='white', pad=10)
            
            # 移除坐标轴
            ax.set_xticks([])
            ax.set_yticks([])
            
            # 添加颜色条
            cbar = plt.colorbar(heatmap, ax=ax)
            cbar.ax.yaxis.set_tick_params(color='white')
            plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='white')
            
            # 创建数值显示
            texts = [[ax.text(j, i, '0',
                            ha='center', va='center',
                            color='white', fontsize=8,
                            fontweight='bold')
                     for j in range(size[1])]
                    for i in range(size[0])]
            
            self.heatmaps[name] = heatmap
            self.texts[name] = texts

    def update(self, frame):
        for name, (finger, size, _) in self.sensors.items():
            data = np.array(self.hand.get_touch_sensor_data(finger)).reshape(size)
            
            # 更新热力图
            self.heatmaps[name].set_array(data)
            
            # 更新数值显示
            for i in range(size[0]):
                for j in range(size[1]):
                    val = int(data[i,j])
                    if val > 0:  # 只显示非零值
                        self.texts[name][i][j].set_text(f'{val}')
                        self.texts[name][i][j].set_color('white')
                    else:
                        self.texts[name][i][j].set_text('')
        
        return list(self.heatmaps.values())

def main():
    viz = TouchSensorVisualizer()
    
    anim = FuncAnimation(viz.fig, 
                        viz.update,
                        interval=30,  
                        blit=True)
    
    plt.suptitle('', 
                 color='white', size=16, y=0.95)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()