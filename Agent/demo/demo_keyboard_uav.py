import keyboard
import airsim

velocity = 1

def callBackFunc(x):
    w = keyboard.KeyboardEvent('down', 28, 'w')             # 前进
    s = keyboard.KeyboardEvent('down', 28, 's')             # 后退
    a = keyboard.KeyboardEvent('down', 28, 'a')             # 左移
    d = keyboard.KeyboardEvent('down', 28, 'd')             # 右移
    up = keyboard.KeyboardEvent('down', 28, 'up')           # 上升
    down = keyboard.KeyboardEvent('down', 28, 'down')       # 下降
    left = keyboard.KeyboardEvent('down', 28, 'left')       # 左转
    right = keyboard.KeyboardEvent('down', 28, 'right')     # 右转
    k = keyboard.KeyboardEvent('down', 28, 'k')             # 获取控制
    l = keyboard.KeyboardEvent('down', 28, 'l')             # 释放控制
    q = keyboard.KeyboardEvent('down', 28, 'q')             # 记录

    if x.event_type == 'down' and x.name == q.name:
        # 记录
        gps = client.getGpsData().gnss.geo_point
        print(f"({gps.longitude},{gps.latitude},{gps.altitude}),")

    elif x.event_type == 'down' and x.name == w.name:
        # 前进
        client.moveByVelocityBodyFrameAsync(velocity, 0, 0, 0.5)
    elif x.event_type == 'down' and x.name == s.name:
        # 后退
        client.moveByVelocityBodyFrameAsync(-velocity, 0, 0, 0.5)
    elif x.event_type == 'down' and x.name == a.name:
        # 左移
        client.moveByVelocityBodyFrameAsync(0, -velocity, 0, 0.5)
    elif x.event_type == 'down' and x.name == d.name:
        # 右移
        client.moveByVelocityBodyFrameAsync(0, velocity, 0, 0.5)
    elif x.event_type == 'down' and x.name == up.name:
        # 上升
        client.moveByVelocityBodyFrameAsync(0, 0, -velocity/2, 0.5)
    elif x.event_type == 'down' and x.name == down.name:
        # 下降
        client.moveByVelocityBodyFrameAsync(0, 0, velocity/2, 0.5)
    elif x.event_type == 'down' and x.name == left.name:
        # 左转
        client.rotateByYawRateAsync(-10*velocity, 0.5)
    elif x.event_type == 'down' and x.name == right.name:
        # 右转
        client.rotateByYawRateAsync(10*velocity, 0.5)
    elif x.event_type == 'down' and x.name == k.name:
        # 无人机起飞
        # get control
        # Async methods returns Future. Call join() to wait for task to complete.
        client.takeoffAsync().join()
    elif x.event_type == 'down' and x.name == l.name:
        # 无人机降落
        client.landAsync().join()
    else:
        # 没有按下按键
        client.hoverAsync().join()  # 第四阶段：悬停秒钟


if __name__ == '__main__':
    # 建立脚本与AirSim环境的连接
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print("get control")
    # unlock
    client.armDisarm(True)
    print("unlock")
    # Async methods returns Future. Call join() to wait for task to complete.
    # 监听键盘事件，执行回调函数
    keyboard.hook(callBackFunc)
    keyboard.wait()