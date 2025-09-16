import tkinter as tk
from agibot_hand import AgibotHandO12

class OmniHandGUI:
    def __init__(self):
        self.hand = AgibotHandO12()
        self.setup_ui()
        self.init_hand()
        
    def setup_ui(self):
        # 主窗口 - 极简风格
        self.root = tk.Tk()
        self.root.title("OmniHand 2025 - Intelligent Control System")
        self.root.geometry("800x600")  # 缩小窗口尺寸
        self.root.configure(bg='#000000')
        
        # 主框架
        main_frame = tk.Frame(self.root, bg='#000000', relief='flat', bd=0)
        main_frame.pack(fill='both', expand=True, padx=12, pady=12)
        
        # 控制面板
        self.setup_control_panel(main_frame)
        
    def setup_control_panel(self, parent):
        # 控制面板
        control_frame = tk.Frame(parent, bg='#0d0d0d', relief='flat', bd=0)
        control_frame.pack(fill='both', expand=True)
        
        # 顶部装饰线
        tech_line = tk.Frame(control_frame, bg='#00d4ff', height=1)
        tech_line.pack(fill='x', padx=0, pady=(0, 0))
        
        # 标题
        title_frame = tk.Frame(control_frame, bg='#0d0d0d', height=40)
        title_frame.pack(fill='x', padx=20, pady=(0, 8))
        
        tk.Label(title_frame, text="关节位置控制", 
                bg='#0d0d0d', fg='#00d4ff', 
                font=('Consolas', 13, 'bold')).pack(anchor='w')
        
        # 滑动条面板
        self.setup_sliders(control_frame)
        
    def setup_sliders(self, parent):
        """设置滑动条控制面板"""
        sliders_frame = tk.Frame(parent, bg='#0d0d0d')
        sliders_frame.pack(fill='both', expand=True, padx=20)
        
        # 初始值
        initial_values = [1990, 201, 3998, 409, 4097, 4096, 2048, 4096, 2038, 4100]
        
        self.position_sliders = []
        for i in range(10):
            # 关节容器
            joint_frame = tk.Frame(sliders_frame, bg='#151515', relief='flat', bd=0)
            joint_frame.pack(fill='x', pady=2)
            
            # 内容框架
            info_frame = tk.Frame(joint_frame, bg='#151515')
            info_frame.pack(fill='x', padx=12, pady=5)
            
            # 关节标签
            joint_label = tk.Label(info_frame, text=f"J{i+1:02d}", 
                                 bg='#151515', fg='#cccccc', 
                                 font=('Consolas', 9, 'bold'), width=4)
            joint_label.pack(side='left')
            
            # 位置滑动条
            slider = tk.Scale(info_frame, from_=0, to=4100, orient='horizontal',
                            bg='#151515', fg='#888888', 
                            troughcolor='#2a2a2a', activebackground='#00d4ff',
                            highlightthickness=0, bd=0, length=170, showvalue=0,
                            command=lambda v, idx=i: self.on_position_slider_change(idx, v))
            slider.set(initial_values[i])
            slider.pack(side='left', padx=(10, 15))
            
            # 数值标签
            value_label = tk.Label(info_frame, text=str(initial_values[i]), 
                                 bg='#151515', fg='#00d4ff', 
                                 font=('Consolas', 9, 'bold'), width=4)
            value_label.pack(side='right')
            
            self.position_sliders.append((slider, value_label))
            
    def init_hand(self):
        """初始化机械手"""
        try:
            init_positions = [1990, 201, 3998, 409, 4097, 4096, 2048, 4096, 2038, 4100]
            self.hand.set_all_joint_positions(init_positions)
            self.update_slider_display(init_positions)
        except Exception as e:
            print(f"初始化失败: {e}")
            
    def on_position_slider_change(self, joint_idx, value):
        """位置滑动条变化回调"""
        self.position_sliders[joint_idx][1].config(text=str(int(float(value))))
        try:
            self.hand.set_joint_position(joint_idx + 1, int(float(value)))
        except Exception as e:
            print(f"关节位置设置失败: {e}")
            
    def update_slider_display(self, positions):
        """更新滑动条显示"""
        for i, (slider, label) in enumerate(self.position_sliders):
            if i < len(positions):
                slider.set(positions[i])
                label.config(text=str(positions[i]))
                
    def run(self):
        self.root.mainloop()

def main():
    gui = OmniHandGUI()
    gui.run()

if __name__ == '__main__':
    main()