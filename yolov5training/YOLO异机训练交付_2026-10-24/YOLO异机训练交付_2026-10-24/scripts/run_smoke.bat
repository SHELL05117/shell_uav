@echo off
REM Smoke run: validate data loading, 6-class ReLU head, GPU and checkpoint saving.
REM 3 epochs, from scratch, seed 42. Produces work/runs/smoke_relu_scratch/weights/last.pt
setlocal
set ROOT=D:\project\yolotraining
set PY=%ROOT%\env\venv-train\Scripts\python.exe
set REPO=%ROOT%\work\repos\yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab
set GIT_PYTHON_REFRESH=quiet
set PATH=%ROOT%\env\venv-train\Scripts;%PATH%
set YOLOv5_AUTOINSTALL=false
cd /d "%REPO%"
"%PY%" train.py --img 640 --batch 16 --epochs 3 --data "%ROOT%\work\dataset\sixclass\data.yaml" --weights "" --cfg models/yolov5s.yaml --workers 4 --seed 42 --device 0 --project "%ROOT%\work\runs" --name smoke_relu_scratch --exist-ok --cache disk
exit /b %ERRORLEVEL%


