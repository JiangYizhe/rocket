# 火箭飞行日志版本库

这个仓库独立管理飞行日志、串口原始转储、解析后的 CSV/SVG/PNG 和日志分析脚本。PCB、Keil 工程和点火代码不放在这个仓库中。

## 目录

- `archive/`：从上级目录导入的各次飞行日志，保留原目录名。
- `captures/`：散落在上级目录的 COM 串口原始转储。
- `external/`：从其他位置导入的压缩日志及其解压内容。
- `archive/legacy_202608/`：从 2026 年 8 月旧版飞控工程中导入的历史日志，`SOURCES.csv` 记录备份与重复映射。
- `analysis/`：可复查的日志分析结果。
- `tools/`：日志导入与索引生成工具。
- `MANIFEST.csv`：每个日志批次的文件数、容量、记录数、最高高度、二级延时和点火时刻。

## 导入新日志

在仓库目录执行：

```powershell
& "C:\Users\91321\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" .\tools\import_logs.py
git add -A
git commit -m "Import flight logs YYYY-MM-DD"
```

导入工具只复制日志，不移动或删除原始目录。它可以重复运行，Git 只会记录真正发生变化的文件。

导入旧版工程内的 2026 年 8 月日志，并重新生成高度预测归零对比表：

```powershell
& "C:\Users\91321\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" .\tools\import_legacy_202608.py
& "C:\Users\91321\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" .\tools\analyze_prediction_dropouts.py
```

## 常用命令

```powershell
git status
git log --oneline --decorate
git diff -- MANIFEST.csv
git show <commit>:MANIFEST.csv
```
