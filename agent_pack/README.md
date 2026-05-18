# WAVE ROVER base chassis AI-agent file pack

This pack is for the Waveshare **WAVE ROVER** base chassis: 4 rubber wheels, four N20 motors, the ESP32-based General Driver for Robots board, no Raspberry Pi, no camera, no wheel encoders.

It is intended for future AI-agent use. The important derived files are:

- `analysis/hardware_summary.md` - overall hardware identity and what is/is not present.
- `analysis/pinout_general_driver.md` - ESP32 pin usage and board signal map.
- `analysis/circuit_blocks.md` - block-level electrical architecture.
- `analysis/mechanical_dimensions.md` - rover and mounting dimensions extracted from Waveshare product images/specs.
- `analysis/command_api_notes.md` - JSON command/API notes for direct control.
- `data/*.yaml`, `data/*.json`, `data/*.csv` - machine-readable summaries.
- `source_files/vendor_downloaded/General_Driver_for_Robots.pdf` - downloaded schematic/circuit diagram PDF.
- `source_files/vendor_reference_images/` - downloaded Waveshare dimension reference images.
- `source_files/rendered_schematic_crops/` - rendered/cropped schematic images used during analysis.
- `scripts/download_vendor_archives.sh` - script for fetching the full vendor archives on a normal machine.

## Important limitation

The full vendor ZIP/RAR archives were located and their URLs are recorded, but this ChatGPT container/tooling blocked saving archive MIME types (`.zip`, `.rar`) directly. Therefore the pack includes the downloaded schematic PDF and dimension images, plus a script and URL manifest to fetch the full raw archives later. Run `scripts/download_vendor_archives.sh` on your own machine to populate `raw_vendor_archives/`.

The most important blocked archive is `WAVE_ROVER_demo.zip`, which should contain the factory Arduino/ESP32 source such as `WAVE_ROVER_v0.9.ino`.
