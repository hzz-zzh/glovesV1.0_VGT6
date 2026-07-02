# 68点触觉手套可视化

1. 双击 `..\run_touch_matrix_viewer.bat`。
2. 在打开的 Chrome 或 Edge 页面中点击“连接串口”。
3. 选择 USART2 对应串口，波特率保持 `921600`。
4. 串口固件输出格式应为：

   `TOUCH seq=... thumb=... index=... middle=... ring=... little=... palm=...`

界面严格按 A0-A3、B0-B3、C0-C3、D0-D3、E0-E3、F0-F47 排列。若触摸后 ADC 值下降，勾选“低值代表按压”。
