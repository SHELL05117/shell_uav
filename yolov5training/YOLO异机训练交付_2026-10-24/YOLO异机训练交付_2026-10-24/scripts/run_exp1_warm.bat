@echo off
REM Experiment 1 (primary): YOLOv5s-ReLU, 6 classes, 640, warm start from the COCO
REM yolov5s.pt checkpoint (ultralytics v7.0). NOTE: that checkpoint was trained with
REM SiLU activations; only shape-matching layers are transferred into the ReLU
REM network, so this is a transfer-learning initialisation, NOT a matched-ReLU
REM pretrained model. Recorded in TRAINING_STATUS.md and the return README.
setlocal
set ROOT=D:\project\yolotraining
set PY=%ROOT%\env\venv-train\Scripts\python.exe
set REPO=%ROOT%\work\repos\yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab
set GIT_PYTHON_REFRESH=quiet
set PATH=%ROOT%\env\venv-train\Scripts;%PATH%
set YOLOv5_AUTOINSTALL=false
cd /d "%REPO%"
"%PY%" train.py --img 640 --batch 16 --epochs 200 --patience 50 --data "%ROOT%\work\dataset\sixclass\data.yaml" --weights "%ROOT%\downloads\yolov5s.pt" --cfg models/yolov5s.yaml --hyp data\hyps\hyp.scratch-low.yaml --workers 4 --seed 42 --device 0 --project "%ROOT%\work\runs" --name exp1_warm_coco_silu --exist-ok --cache disk
exit /b %ERRORLEVEL%
