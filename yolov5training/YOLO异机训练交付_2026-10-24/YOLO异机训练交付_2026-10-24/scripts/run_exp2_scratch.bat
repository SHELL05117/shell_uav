@echo off
REM Experiment 2 (ablation): same recipe as exp1 but random initialisation, to show
REM what is lost without any pretrained weights (the documented limitation of this
REM handoff: no verified YOLOv5s-ReLU pretrained checkpoint is available).
setlocal
set ROOT=D:\project\yolotraining
set PY=%ROOT%\env\venv-train\Scripts\python.exe
set REPO=%ROOT%\work\repos\yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab
set GIT_PYTHON_REFRESH=quiet
set PATH=%ROOT%\env\venv-train\Scripts;%PATH%
set YOLOv5_AUTOINSTALL=false
cd /d "%REPO%"
"%PY%" train.py --img 640 --batch 16 --epochs 200 --patience 50 --data "%ROOT%\work\dataset\sixclass\data.yaml" --weights "" --cfg models/yolov5s.yaml --hyp data\hyps\hyp.scratch-low.yaml --workers 4 --seed 42 --device 0 --project "%ROOT%\work\runs" --name exp2_scratch_relu --exist-ok --cache disk
exit /b %ERRORLEVEL%
