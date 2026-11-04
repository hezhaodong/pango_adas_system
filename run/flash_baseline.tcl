cfg_connect -ip 127.0.0.1 -port 65420
catch {cfg_remove_all_devices}
cfg_scan_chain
cfg_assign_file -file D:/Users/daaa/high_FINAL_2026-10-28/run/BASELINE_hdmi_loop_E819B0EF.sbit -device_index 0
catch {cfg_program -device_index 0}
puts PROGRAM_OK
