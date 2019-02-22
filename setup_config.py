# coding=utf-8
import odrive

odrv0 = odrive.find_any()
odrv0.axis0.motor.config.pole_pairs = 5
odrv0.axis0.motor.config.calibration_current = 2
odrv0.axis0.motor.config.resistance_calib_max_voltage = 2.4
odrv0.axis0.motor.config.current_lim = 5
odrv0.axis0.encoder.config.cpr = 4096
odrv0.save_configuration()
