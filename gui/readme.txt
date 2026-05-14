**How to run the Selfie Drawing Robot GUI starter**

You need:

* Python 3 installed
* a webcam
* the file `selfie_drawing_gui_starter.py`

## On Windows

1. Put `selfie_drawing_gui_starter.py` in a folder.
2. Open that folder in VS Code, or open PowerShell in that folder.
3. Create a virtual environment:

```powershell
py -m venv .venv
```

4. Activate it:

```powershell
.venv\Scripts\Activate
```

If PowerShell blocks it, run:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

Then activate again:

```powershell
.venv\Scripts\Activate
```

5. Install the required packages:

```powershell
pip install PySide6 opencv-python
```

6. Run the program:

```powershell
python selfie_drawing_gui_starter.py
```

If `python` does not work, try:

```powershell
py selfie_drawing_gui_starter.py
```

---

## On Ubuntu

1. Put `selfie_drawing_gui_starter.py` in a folder.
2. Open a terminal in that folder.
3. Make sure Python venv support is installed:

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip
```

4. Create a virtual environment:

```bash
python3 -m venv .venv
```

5. Activate it:

```bash
source .venv/bin/activate
```

6. Install the required packages:

```bash
pip install PySide6 opencv-python
```

7. Run the program:

```bash
python3 selfie_drawing_gui_starter.py
```

---

## What should happen

The GUI window should open and show:

* live webcam preview
* Capture button
* Retake button
* Process button
* Start Drawing button
* a fake/simulated drawing progress bar

At the moment:

* the preview is generated locally in the GUI
* the drawing progress is simulated
* ROS2 is **not connected yet**

---

## If the webcam does not work

Common fixes:

* close any other app using the camera
* check OS camera permissions
* try changing the camera index in the code

Find this line:

```python
self.camera = CameraHandler()
```

and change the class default index or update the constructor to use another camera, for example:

```python
self.camera = CameraHandler(camera_index=1)
```

If `1` does not work, try `2`.

---

## If package install fails

Try upgrading pip first:

```bash
python3 -m pip install --upgrade pip
```

or on Windows:

```powershell
python -m pip install --upgrade pip
```

Then install again:

```bash
pip install PySide6 opencv-python
```

---

## Important note

This is just the GUI prototype.
When the ROS2 nodes are ready, the fake processing and fake drawing progress will be replaced with real communication to the perception and motion planning nodes.
