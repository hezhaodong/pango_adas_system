# YOLOv8 crosswalk / vehicle / pedestrian integration

Key `7` toggles the failure-to-yield feature backed by the legacy YOLOv8
traffic model imported from `ARM_final1`. Key `5` shares the same loaded
backend for pedestrian-crossing detection, so the model is never loaded or
run twice when both features are enabled.

The application searches for these files in `/userdata/model`,
`/mnt/usb/model`, and the application working directory:

- `yolov8_traffic.rknn`
- `yolov8_traffic_labels_list.txt`

The ARM_final1 behaviour is retained:

- box threshold: `0.50`;
- NMS threshold: `0.45`;
- pedestrian/rider/motor boxes are expanded to three times their original
  width for overlap decisions and display;
- detecting a crosswalk opens a one-second person/vehicle decision window;
- a detected failure-to-yield warning remains visible for three seconds.

Both the normal Qt window and the EGLFS full-screen window map key `7` to this
toggle. Key `6` starts YOLOPv2 and Detect3D together as the automatic-driving
assistance feature.
