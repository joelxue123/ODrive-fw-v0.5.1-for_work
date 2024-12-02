import numpy as np
from scipy.signal import filtfilt, butter
import matplotlib.pyplot as plt

class FlexibleJointSystem:
    def __init__(self, params):
        self.J_m = params['J_m']
        self.J_l = params['J_l']
        self.K_s = params['K_s']
        self.B = params['B']
        self.Ts = params['Ts']
        
        # 计算系统特性
        self.omega_n = np.sqrt(self.K_s/self.J_l)
        self.zeta = self.B/(2*np.sqrt(self.K_s*self.J_l))
        
    def calculate_acceleration(self, pos):
        """使用中心差分计算加速度"""
        acc = np.zeros_like(pos)
        acc[1:-1] = (pos[2:] - 2*pos[1:-1] + pos[:-2])/self.Ts**2
        acc[0] = acc[1]
        acc[-1] = acc[-2]
        return acc
    
    def compute_feedforward(self, theta_l):
        """计算前馈补偿"""
        acc_l = self.calculate_acceleration(theta_l)
        # 降低滤波器截止频率，使信号更平滑
        b, a = butter(2, 0.05)
        acc_l_filtered = filtfilt(b, a, acc_l)
        compensation = (self.J_l/self.K_s) * acc_l_filtered
        theta_m = theta_l + compensation
        return theta_m, compensation

class FeedforwardController:
    def __init__(self, sys_params):
        self.system = FlexibleJointSystem(sys_params)
        self.Ts = sys_params['Ts']
        
    def smooth_trajectory(self, traj, cutoff_freq=10.0):  # 降低截止频率
        nyq = 1/(2*self.Ts)
        b, a = butter(2, cutoff_freq/nyq)
        return filtfilt(b, a, traj)
    
    def generate_command(self, theta_l_d):
        theta_l_filtered = self.smooth_trajectory(theta_l_d)
        theta_m, compensation = self.system.compute_feedforward(theta_l_filtered)
        vel_cmd = np.gradient(theta_m, self.Ts)
        vel_cmd = self.smooth_trajectory(vel_cmd, cutoff_freq=8.0)  # 降低截止频率
        return theta_m, vel_cmd, compensation

def generate_trajectory(t):
    """生成更平滑的轨迹"""
    tf = t[-1]
    normalized_t = t/tf
    # 降低运动幅度，使用更平滑的多项式
    traj = 1.0 * (35*normalized_t**4 - 84*normalized_t**5 + 70*normalized_t**6 - 20*normalized_t**7)
    return traj

def simulate_system_response(motor_pos, system_params):
    """改进的系统响应模拟"""
    K_s = system_params['K_s']
    J_l = system_params['J_l']
    B = system_params['B']
    Ts = system_params['Ts']
    
    # 初始化状态
    pos = np.zeros_like(motor_pos)
    vel = np.zeros_like(motor_pos)
    
    # 使用改进的欧拉法进行数值积分
    for i in range(1, len(motor_pos)):
        # 计算当前时刻的力矩
        torque = K_s * (motor_pos[i-1] - pos[i-1])
        damping = B * vel[i-1]
        
        # 计算加速度
        acc = (torque - damping) / J_l
        
        # 更新状态（使用改进的欧拉法）
        vel[i] = vel[i-1] + acc * Ts
        pos[i] = pos[i-1] + (vel[i-1] + vel[i])/2 * Ts
        
        # 添加限幅以增加数值稳定性
        vel[i] = np.clip(vel[i], -100, 100)
        
    return pos

def analyze_error(desired, actual):
    error = desired - actual
    max_error = np.max(np.abs(error))
    rms_error = np.sqrt(np.mean(error**2))
    return {
        'max_error': max_error,
        'rms_error': rms_error,
        'error': error
    }

def main():
    # 调整系统参数
    params = {
        'J_m': 0.01,    # 电机转动惯量
        'J_l': 0.05,    # 负载转动惯量
        'K_s': 100,     # 降低刚度
        'B': 1.0,       # 增加阻尼
        'Ts': 0.001     # 采样时间
    }
    
    controller = FeedforwardController(params)
    
    t = np.arange(0, 5, params['Ts'])
    theta_l_desired = generate_trajectory(t)
    
    pos_cmd, vel_cmd, compensation = controller.generate_command(theta_l_desired)
    
    end_pos = simulate_system_response(pos_cmd, params)
    
    error_stats = analyze_error(theta_l_desired, end_pos)
    
    # 绘图
    plt.figure(figsize=(15, 12))
    
    plt.subplot(411)
    plt.plot(t, theta_l_desired, 'b-', label='Desired')
    plt.plot(t, pos_cmd, 'r--', label='Motor Command')
    plt.plot(t, end_pos, 'g:', label='Actual')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Position (rad)')
    plt.title(f'Flexible Joint Control (Max Error: {error_stats["max_error"]:.4f}, RMS Error: {error_stats["rms_error"]:.4f})')
    
    plt.subplot(412)
    plt.plot(t, vel_cmd, 'r-', label='Motor Velocity')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Velocity (rad/s)')
    
    plt.subplot(413)
    plt.plot(t, compensation, 'b-', label='Compensation')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Compensation (rad)')
    
    plt.subplot(414)
    plt.plot(t, error_stats['error'], 'r-', label='Error')
    plt.grid(True)
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Error (rad)')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()