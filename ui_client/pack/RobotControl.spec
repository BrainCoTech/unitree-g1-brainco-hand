# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path


PACK_DIR = Path(SPECPATH).resolve()
CLIENT_DIR = PACK_DIR.parent

a = Analysis(
    [str(CLIENT_DIR / 'gui_start.py')],
    pathex=[str(CLIENT_DIR)],
    binaries=[],
    datas=[
        (str(CLIENT_DIR / 'config' / 'gui_config.yaml'), '.'),
    ],
    hiddenimports=[],
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
    name='RobotControl',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
