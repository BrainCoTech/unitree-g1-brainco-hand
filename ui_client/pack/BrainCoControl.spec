# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

from PyInstaller.utils.hooks import collect_submodules


PACK_DIR = Path(__file__).resolve().parent
CLIENT_DIR = PACK_DIR.parent


a = Analysis(
    [str(CLIENT_DIR / "ui_test.py")],
    pathex=[str(CLIENT_DIR)],
    binaries=[],
    datas=[
        (str(CLIENT_DIR / "client_config.yaml"), "."),
        (str(CLIENT_DIR / "gui_config.yaml"), "."),
    ],
    hiddenimports=collect_submodules("robot_agent"),
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="BrainCoControl",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    # Windows architecture follows the Python interpreter used to build.
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
