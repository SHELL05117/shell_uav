@echo off
REM Experiment 3: fine-tune the round-1 model with rotation / scale augmentation.
REM Motivation: at the 0.25 working point the round-1 model confused several symbols
REM (see work/reports/eval_exp1_test/task_failures.json); the markers are photographed
REM at strong in-plane rotation and round 1 trained with degrees=0.
setlocal
set ROOT=D:\project\yolotraining
set PY=%ROOT%\env\venv-train\Scripts\python.exe
set REPO=%ROOT%\work\repos\yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab
set GIT_PYTHON_REFRESH=quiet
set PATH=%ROOT%\env\venv-train\Scripts;%PATH%
set YOLOv5_AUTOINSTALL=false
cd /d "%REPO%"
"%PY%" train.py --img 640 --batch 16 --epochs 120 --patience 30 --data "%ROOT%\work\dataset\sixclass\data.yaml" --weights "%ROOT%\work\runs\exp1_warm_coco_silu\weights\best.pt" --hyp "%ROOT%\work\hyp\hyp_rot_finetune.yaml" --workers 3 --seed 42 --device 0 --project "%ROOT%\work\runs" --name exp3_rot_finetune --exist-ok --cache disk
exit /b %ERRORLEVEL%
