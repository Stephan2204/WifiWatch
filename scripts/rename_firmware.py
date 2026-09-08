Import("env")
from pathlib import Path
import shutil
import subprocess


def _find_boot_app0(env):
    """Locate Arduino's initial OTA-data image when the framework provides it."""
    platform = env.PioPlatform()
    for package_name in ("framework-arduinoespressif32", "framework-arduinoespressif32-libs"):
        package_dir = platform.get_package_dir(package_name)
        if not package_dir:
            continue
        candidates = [
            Path(package_dir) / "tools" / "partitions" / "boot_app0.bin",
            Path(package_dir) / "partitions" / "boot_app0.bin",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
    return None


def _merge_factory(env, output_path, app_bin):
    build_dir = Path(env.subst("$BUILD_DIR"))
    python_exe = env.subst("$PYTHONEXE")

    chip = env.GetProjectOption("custom_factory_chip")
    bootloader_offset = env.GetProjectOption("custom_bootloader_offset")
    app_offset = env.GetProjectOption("custom_app_offset", "0x10000")

    bootloader = build_dir / "bootloader.bin"
    partitions = build_dir / "partitions.bin"
    boot_app0 = _find_boot_app0(env)

    required = [bootloader, partitions, app_bin]
    missing = [str(p) for p in required if not p.exists()]
    if missing:
        print("WiFiWatch Factory image skipped; missing: " + ", ".join(missing))
        return False

    platform = env.PioPlatform()
    esptool_dir = platform.get_package_dir("tool-esptoolpy")
    esptool_script = None
    if esptool_dir:
        candidate = Path(esptool_dir) / "esptool.py"
        if candidate.exists():
            esptool_script = candidate

    if esptool_script:
        cmd = [python_exe, str(esptool_script)]
    else:
        # Fallback for platforms where esptool is installed as a Python module.
        cmd = [python_exe, "-m", "esptool"]

    cmd += [
        "--chip", chip,
        "merge_bin",
        "-o", str(output_path),
        bootloader_offset, str(bootloader),
        "0x8000", str(partitions),
    ]

    # boot_app0 initializes the OTA-data slot for Arduino OTA layouts.
    if boot_app0 and boot_app0.exists():
        cmd += ["0xe000", str(boot_app0)]

    cmd += [app_offset, str(app_bin)]

    print("WiFiWatch Factory merge:", " ".join(cmd))
    subprocess.check_call(cmd)
    return output_path.exists()


def create_release_images(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    build_dir = Path(env.subst("$BUILD_DIR"))
    progname = env.subst("$PROGNAME")
    app_bin = build_dir / f"{progname}.bin"

    base_name = env.GetProjectOption("custom_firmware_base")
    dst_dir = project_dir / "firmware"
    dst_dir.mkdir(parents=True, exist_ok=True)

    ota_path = dst_dir / f"{base_name}-OTA.bin"
    factory_path = dst_dir / f"{base_name}-Factory.bin"

    if not app_bin.exists():
        print(f"WiFiWatch release images skipped: {app_bin} not found")
        return

    shutil.copy2(app_bin, ota_path)
    print(f"WiFiWatch OTA firmware:     {ota_path}")

    if _merge_factory(env, factory_path, app_bin):
        print(f"WiFiWatch Factory firmware: {factory_path}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", create_release_images)
