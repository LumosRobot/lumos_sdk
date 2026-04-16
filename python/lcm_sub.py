#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import lcm
# 导入你生成的 LCM 消息类型
from lcm_typedef.sdk_lcmt_joint_cmds import sdk_lcmt_joint_cmds

# ==================== 回调函数：收到消息就进来 ====================
def handler(channel, data):
    """
    LCM 消息回调
    channel: 话题名
    data: 原始字节
    """
    # 解码消息
    msg = sdk_lcmt_joint_cmds.decode(data)
    
    print("=" * 50)
    print(f"📥 收到 LCM 话题: {channel}")
    
    # ==================== 打印你需要的字段 ====================
    # 根据你的消息结构自己加字段
    print(f"时间戳: {msg.timestamp}")
    # print(f"关节数量: {msg.num_joints}")
    # print(f"位置: {msg.position}")
    # print(f"速度: {msg.velocity}")
    # print(f"力矩: {msg.effort}")

# ==================== 主函数 ====================
if __name__ == "__main__":
    # 创建 LCM 实例
    lc = lcm.LCM()
    
    # 订阅 LCM 话题
    # 把 "YOUR_CHANNEL" 换成你真实的话题名！
    channel = "YOUR_CHANNEL"  # 例如：joint_cmds, robot_cmd 等
    subscription = lc.subscribe(channel, handler)
    
    print(f"✅ LCM 订阅已启动，监听话题: {channel}")
    print("按 Ctrl+C 退出\n")
    
    # 循环监听消息
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        print("\n🛑 退出订阅")
