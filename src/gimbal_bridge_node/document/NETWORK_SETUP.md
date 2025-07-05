# 有线局域网IP配置指南

## 临时修改IP地址
```bash
sudo ip addr add 192.168.144.100/24 dev eth0
sudo ip link set eth0 up
```

## 永久修改IP地址（开机自动运行）

1. 创建systemd服务文件：
```bash
sudo bash -c 'cat > /etc/systemd/system/set-eth0-ip.service <<EOF
[Unit]
Description=Set static IP for eth0
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip addr add 192.168.144.100/24 dev eth0
ExecStart=/sbin/ip link set eth0 up

[Install]
WantedBy=multi-user.target
EOF'
```

2. 启用服务：
```bash
sudo systemctl enable set-eth0-ip.service
```

## 验证IP配置
```bash
ip addr show eth0
```

## 注意事项
- 确保eth0是有线网卡接口名
- 重启后IP配置会自动生效
- 如需恢复DHCP，请禁用服务并删除配置文件
