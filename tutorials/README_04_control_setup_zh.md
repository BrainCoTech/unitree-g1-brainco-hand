## Host PC Setup

Download the `ui_client` directory to the host computer.

Install the client-side dependencies
```sh
# create a new environment for the control GUI
conda create -n braincogui python=3.10
conda activate braincogui
pip install PySide6 PyYAML jinja2 typeguard
```