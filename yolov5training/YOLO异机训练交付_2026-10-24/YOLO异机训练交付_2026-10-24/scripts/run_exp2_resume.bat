@echo off
REM Experiment 2 resume: continue the from-scratch ablation from its last checkpoint.
REM The first attempt died at epoch ~39 in a DataLoader worker when the machine was
REM also running evaluation jobs; workers are reduced to 2 here.
setlocal
set ROOT=D:\project\yolotraining
set PY=%ROOT%\env\venv-train\Scripts\python.exe
set REPO=%ROOT%\work\repos\yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab
set GIT_PYTHON_REFRESH=quiet
set PATH=%ROOT%\env\venv-train\Scripts;%PATH%
set YOLOv5_AUTOINSTALL=false
cd /d "%REPO%"
"%PY%" train.py --resume "%ROOT%\work\runs\exp2_scratch_relu\weights\last.pt" --workers 2
exit /b %ERRORLEVEL%
