# YOLOPv2 ARM integration

The former 80-class `yolov8detect` control now loads YOLOPv2. Existing Qt
worker, UI toggle, remote `yoloOn`/`yoloOff`, frame scheduling, and result
overlay connections are retained, so callers do not need a protocol change.

YOLOPv2 provides three results in one inference:

- COCO 80-class axis-aligned traffic/object boxes;
- drivable-area segmentation (green translucent overlay);
- lane-line segmentation (red overlay).

The separate three-class traffic-light/crosswalk shortcut and the helmet
detector remain independent. The shared detector automatically selects the
legacy YOLOv8 DFL decoder for those legacy models.

## Model deployment

Use the already validated model:

`yolopv2_384x640_fp16.rknn`

Expected SHA-256:

`ed0dc1a0e662bad5a5b1ffec3d75717ef881b0782a54b491e1032a1f06cce274`

The application prefers the board-validated INT8 model and falls back to FP16.
It searches, in order:

1. `/userdata/model/yolopv2_384x640_int8.rknn`
2. `/mnt/usb/model/yolopv2_384x640_int8.rknn`
3. `./yolopv2_384x640_int8.rknn`
4. `/userdata/model/yolopv2_384x640_fp16.rknn`
5. `/mnt/usb/model/yolopv2_384x640_fp16.rknn`
6. `./yolopv2_384x640_fp16.rknn`
7. `/mnt/usb/ARM_final8/yolopv2_384x640_fp16.rknn` (compatibility fallback)

Recommended board deployment:

```sh
mkdir -p /userdata/model
cp ./yolopv2_384x640_int8.rknn /userdata/model/
cp ./yolopv2_384x640_fp16.rknn /userdata/model/  # optional fallback
sha256sum /userdata/model/yolopv2_384x640_int8.rknn
```

## Rebuild and test on the board

Because `yolov8_detector.cpp`, headers, and UI code changed, a rebuild is
required. Regenerate the Makefile so Qt's dependency information is fresh:

```sh
cd /mnt/usb/ARM_final1.4
qmake camera_display.pro
make -j2
./camera_display
```

If the board is memory-constrained, use `make -j1`. After camera startup,
press `6` to enable the automatic-driving assistance feature.
Successful startup prints `YOLOPv2 loaded: 640x384, outputs=5`; the image
should show boxes plus green drivable-area and red lane overlays.

The tested runtime combination is RKNN API 2.3.0 with driver 0.9.8. The same
embedded image took about 695 ms with FP16 and 228.291 ms with INT8. Both runs
produced 20 detections; INT8 is therefore the default while FP16 remains a
filename-based fallback. The asynchronous/latest-frame scheduling is retained.
