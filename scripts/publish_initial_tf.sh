if [ "$#" -ne 1 ]; then
    echo "Usage: $0 FIXED_FRAME(odom/map/...)"
    exit
fi
rosrun tf2_ros static_transform_publisher 0.935 1.34 -0.023 0.001 -0.003 0.737 0.676 mocap $1
