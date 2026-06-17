import os
import re
import shutil
Import("env")

def rename_firmware(source, target, env):
    # --- Read version from main.cpp ---
    main_cpp = os.path.join(env.subst("$PROJECT_DIR"), "src", "main.cpp")

    firmware_version = "unknown"
    device_id        = "DEVICE"

    try:
        # ✅ FIX: use utf-8 with errors='ignore' to handle non-ASCII bytes
        with open(main_cpp, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()

        ver_match = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"', content)
        dev_match = re.search(r'#define\s+DEVICE_ID\s+"([^"]+)"', content)

        if ver_match:
            firmware_version = ver_match.group(1)
        if dev_match:
            device_id = dev_match.group(1)

    except Exception as e:
        print(f"[rename_firmware] WARNING: Could not read main.cpp: {e}")

    # --- Locate the built .bin file ---
    # ✅ FIX: use env.subst() to properly expand PlatformIO variables
    build_dir  = env.subst("$BUILD_DIR")
    source_bin = os.path.join(build_dir, "firmware.bin")

    if not os.path.exists(source_bin):
        print(f"[rename_firmware] WARNING: firmware.bin not found at {source_bin}")
        return

    # --- Build output filename ---
    # Output: firmware/<DEVICE_ID>_<FIRMWARE_VERSION>.bin
    output_dir = os.path.join(env.subst("$PROJECT_DIR"), "firmware")
    os.makedirs(output_dir, exist_ok=True)

    dest_name = f"{device_id}_{firmware_version}.bin"
    dest_bin  = os.path.join(output_dir, dest_name)

    shutil.copy(source_bin, dest_bin)

    print("")
    print("=" * 50)
    print(f"[rename_firmware] Firmware built successfully!")
    print(f"[rename_firmware] Device  : {device_id}")
    print(f"[rename_firmware] Version : {firmware_version}")
    print(f"[rename_firmware] Output  : firmware/{dest_name}")
    print("=" * 50)
    print("")

env.AddPostAction("$BUILD_DIR/firmware.bin", rename_firmware)