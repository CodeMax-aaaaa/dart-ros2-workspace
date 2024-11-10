import mido


# MIDI 音高对应的频率转换函数
def midi_to_freq(midi_note):
    return 440 * (2 ** ((midi_note - 69) / 12))


# 选择音轨并转换为 {频率, 持续时间} 格式
def convert_track_to_notes(mid, track_index):
    # track = mid.tracks[track_index]
    notes = []
    current_time = 0  # 当前绝对时间
    last_note_end_time = 0  # 上一个音符的结束时间
    is_note_playing = False  # 是否有音符正在播放

    for msg in mid.play():
        current_time += msg.time

        # 当没有音符播放且时间间隔不为零时，记录一个 {0, 持续时间}
        if not is_note_playing and current_time - last_note_end_time > 0:
            silence_duration = current_time - last_note_end_time
            notes.append((0, silence_duration * 1000))

        # 处理 'note_on' 消息
        if msg.type == "note_on" and msg.velocity > 0:
            freq = midi_to_freq(msg.note)  # 将 MIDI 音符转换为频率
            duration = msg.time  # 持续时间是从上一个消息的时间差
            notes.append((round(freq), duration * 1000))
            is_note_playing = True
            last_note_end_time = current_time

        # 处理 'note_off' 或 'note_on' 消息（velocity 为 0 表示音符关闭）
        elif (msg.type == "note_off") or (msg.type == "note_on" and msg.velocity == 0):
            is_note_playing = False
            last_note_end_time = current_time

        print(msg)

    return notes


def main():
    # 读取 MIDI 文件
    midi_file_path = input("请输入 MIDI 文件的路径: ")
    mid = mido.MidiFile(midi_file_path)

    # 显示可用音轨
    print("\n可用音轨列表:")
    for i, track in enumerate(mid.tracks):
        print(f"音轨 {i}: {track.name or 'Unnamed Track'}")

    # 用户选择音轨
    track_index = int(input("\n选择要处理的音轨编号: "))

    # 转换音符为 {频率, 持续时间} 格式
    notes = convert_track_to_notes(mid, track_index)

    # 打印结果
    print("\n转换后的音符信息:")
    for freq, duration in notes:
        print(f"{{{freq}, {int(duration)}}},")


if __name__ == "__main__":
    main()
