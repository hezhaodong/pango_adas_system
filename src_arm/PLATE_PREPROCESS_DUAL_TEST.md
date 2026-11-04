# SCI + MIMO+ RKNN board test

This is a standalone plate-crop test. It does not modify the original video
pipeline.

## U-disk layout

```text
/userdata/model/sci_plate_32x2_fp16_rk3568.rknn
/userdata/model/mimo_plus_stage5_188x48_fp16_rk3568.rknn
/userdata/model/bluespecial.rknn
```

The input path must contain plate images already cropped by YOLO. Full vehicle
frames must not be passed directly to the enhancement models.

## Compile on RK3568

Direct `g++` build:

```bash
cd /mnt/usb/ARM_final1
g++ -O3 -std=gnu++17 -I. -I/usr/include/opencv4 \
  plate_preprocess_dual_test.cpp plate_preprocess_rknn.cpp lprnet.cc \
  -o plate_preprocess_dual_test \
  -lrknnrt -ldl -pthread \
  -lopencv_core -lopencv_imgproc -lopencv_imgcodecs
```

Or use qmake:

```bash
cd /mnt/usb/ARM_final1
qmake plate_preprocess_dual_test.pro
make -j4
```

## Run

Single image:

```bash
./plate_preprocess_dual_test \
  --input /mnt/usb/test_plates/plate.jpg
```

Directory, first 100 images:

```bash
./plate_preprocess_dual_test \
  --input /mnt/usb/extracted_datasets_split_v4/ccpd_blur/test_images \
  --limit 1000 --seed 20260803 --warmup 5 --repeat 1 --progress 100 --cascade
```

Also test SCI followed by MIMO+:

```bash
./plate_preprocess_dual_test \
  --input /mnt/usb/extracted_datasets_split_v4/ccpd_blur/test_images \
  --limit 100 --cascade
```

When the input is a standard `test_images` directory, results are separated by
subset automatically:

```text
/mnt/usb/plate_preprocess_results/ccpd_blur
/mnt/usb/plate_preprocess_results/ccpd_db
```

When `--limit` is smaller than the dataset, images are sampled with a
deterministic PRNG controlled by `--seed`; they are not the first N sorted
files. Passing an explicit `--output-dir` disables automatic subset naming.

For an input named `test_images`, the program automatically loads
`../test.txt`. It recognizes raw, SCI, MIMO+, and (with `--cascade`) SCI->MIMO
images using bluespecial. The console and CSV report exact full-plate accuracy
for all branches, in addition to preprocessing time, NPU-reported time,
brightness, and Laplacian variance.

Use `--labels /path/to/test.txt` for a non-standard dataset layout. Use
`--no-lpr` to keep the original speed/image-only test. LPR's verbose per-image
logging is suppressed by default; pass `--verbose-lpr` when debugging one image.

Model paths can be overridden without recompiling:

```bash
./plate_preprocess_dual_test --input /mnt/usb/test_plates \
  --sci-model /userdata/model/my_sci.rknn \
  --mimo-model /userdata/model/my_mimo.rknn \
  --lpr-model /userdata/model/my_bluespecial.rknn
```
