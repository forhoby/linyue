# 音频数据消息
# 包含音频元数据和原始PCM数据

# 标准消息头（包含时间戳和帧ID）
std_msgs/Header header

# 采样率（Hz）
uint32 sample_rate

# 声道数（1=单声道，2=立体声）
uint8 channels

# 音频格式
# 与ALSA格式值一致
# 1: U8 (8位无符号) - SND_PCM_FORMAT_U8
# 2: S16_LE (16位有符号小端) - SND_PCM_FORMAT_S16_LE
# 6: S24_LE (24位有符号小端) - SND_PCM_FORMAT_S24_LE
# 10: S32_LE (32位有符号小端) - SND_PCM_FORMAT_S32_LE
# 14: FLOAT32_LE (32位浮点小端) - SND_PCM_FORMAT_FLOAT_LE
uint8 format

# 每个采样的字节数
uint8 bytes_per_sample

# 原始音频数据（PCM）
uint8[] data
