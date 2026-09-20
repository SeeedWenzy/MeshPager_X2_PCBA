在vscode中的ESP-IDF Terminal 中输入以下命令保存日志导文件夹下：

mkdir logs
$log = "logs\monitor_$(Get-Date -Format 'yyyyMMdd_HHmmss').log"
idf.py monitor -b 115200 --timestamps --force-color | Tee-Object -FilePath $log