# 自定义消息类型，包含一个时间戳、一个二维像素点、一个字符串、一个坐标系ID和像素尺寸
time timestamp  # 时间戳
int32 x  # 二维像素点的 x 坐标
int32 y  # 二维像素点的 y 坐标
string message  # 字符串消息
string coordinate_system_id  # 坐标系ID
int32 width  # 像素宽度
int32 height  # 像素高度
