if [ "$1" = "wlan0" ]; then
    # 移除默认路由
    echo "Using wlan0. Delete the default route."
    sudo ip route del default via 192.168.123.1 dev eth0
else
    echo "Using eth0."
fi

# 显示正在使用的默认路由（metric 最小）
active_route=$(ip route show default | sort -k5 -n | head -n1)

echo "Active route: $active_route"

# 启动一次系统时间同步服务
echo "Restarting the system time sync service and admin privileges...."
sudo systemctl restart systemd-timesyncd
# 显示当前时间状态
date