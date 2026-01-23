#!/bin/bash
# 检查 UVC 相机设备并显示 USB 路径信息

echo "=========================================="
echo "UVC 相机设备检查"
echo "=========================================="
echo ""

# 检查 /dev/video* 设备
echo "📹 检测到的视频设备:"
echo ""

for dev in /dev/video*; do
    if [ -e "$dev" ]; then
        echo "设备: $dev"
        echo "  序列号: $(udevadm info --query=property --name="$dev" 2>/dev/null | grep 'ID_SERIAL_SHORT=' | cut -d'=' -f2)"
        echo "  USB路径: $(udevadm info --query=property --name="$dev" 2>/dev/null | grep 'ID_PATH=' | cut -d'=' -f2)"
        echo "  设备路径: $(udevadm info --query=property --name="$dev" 2>/dev/null | grep 'DEVPATH=' | cut -d'=' -f2)"
        echo ""
    fi
done

echo "=========================================="
echo "当前 udev 规则映射:"
echo "=========================================="
if [ -d "/dev/uvc_camera" ]; then
    ls -la /dev/uvc_camera/
else
    echo "❌ /dev/uvc_camera/ 目录不存在"
    echo "   说明 udev 规则没有创建符号链接"
fi

echo ""
echo "=========================================="
echo "建议操作:"
echo "=========================================="
echo "1. 如果 USB 路径改变了，需要更新 udev 规则:"
echo "   sudo nano prototype/bash/99-uvc-cameras.rules"
echo ""
echo "2. 重新加载 udev 规则:"
echo "   sudo udevadm control --reload-rules"
echo "   sudo udevadm trigger"
echo ""
echo "3. 检查符号链接是否创建:"
echo "   ls -la /dev/uvc_camera/"
