-- ------------------- --
-- AtiIface properties --
-- ------------------- --

-- Low-pass filter properties.
lowpass_filter_on = true
filter_bandwidth = 50 -- [Hz]
filter_order = 2 -- 1st or 2nd.
compensate_deadband = true
f_deadband = 0.01 -- [N]
m_deadband = 0 -- [Nm]

-- ATI scale- and calibration matrix. These are sensor dependent (in this case the sensor number is FT09249, an SI-125-03 sensor).
scale_matrix = rtt.Variable("array")
scale_matrix:resize(16)
scale_matrix:fromtab{   0.384871821,    0,              0,              0,              0,              0,
                        0,              0.384871821,    0,              0,              0,              0,
                        0,              0,              1.289512556,    0,              0,              0,
                        0,              0,              0,              0.007369723,    0,              0,
                        0,              0,              0,              0,              0.007369723,    0,
                        0,              0,              0,              0,              0,              0.006061377 }

calibration_matrix = rtt.Variable("array")
calibration_matrix:resize(16)
calibration_matrix:fromtab{ 0.36204,    -0.03265,   -0.40493,   34.49599,   -0.38315,   -34.40333,
                            1.43484,    -39.81456,  -0.36320,   19.93590,   -0.61420,   19.67878,
                            19.25555,   -0.86777,   19.74649,   -0.73087,   19.68441,   -1.32588,
                            -0.21052,   0.06228,    33.47953,   -1.29460,   -33.38817,  2.47414,
                            -39.58429,  2.03419,    18.65949,   -0.93029,   20.67007,   -1.29452,
                            0.11240,    -19.84522,  0.63272,    -19.77099,  0.46242,    -19.97687   }
