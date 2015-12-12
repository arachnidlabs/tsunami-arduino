The purpose of the Calibration sketch in the Tsunami library is to interactively
generate - and persistently store - corrective calibration coefficients for two 
of Tsunami's output settings (the generated waveform's offset and amplitude) and
three of its input measurements (the measured mean value, peak value and current
value of the input). Accordingly, this is accomplished in five distinct phases,
the first two (related to the output) requiring a generic digital multimeter to
measure and enter actually generated output values; for the final three phases
the input needs to be connected to the output with a short loop cable or wire,
while the sketch automatically takes readings of the previously measured output
values to calibrate the input as well. Finally, a list of suggested calibration
coefficients is produced offering the chance to store them in the non-volatile
EEPROM memory of the Tsunami making the calibration available to other sketches.
Beyond its main purpose as an interactive calibration tool, the sketch can also
be used to inspect, manually edit or reset calibration coefficients.

It is important to note that in its current form calibration is NOT effective
automatically, but needs to be explicitly loaded in the user sketch via one or
more "useCalibrationData()" calls; the choices are to selectively load any of
the individual calibration channels (offset, amplitude, mean value, peak value
and current value), or all of them or none at all (unload all calibration data)
Loading calibration data is risk free in the sense that absence of calibration
values is detected by the library, resulting in an error being returned and no
correction being applied.

The calibration model assumes and corrects for a strictly linear error, such as
a gain and / or offset error applied to the ideal value, and therefore corrects
neither errors of non-linearity (if any exist) nor errors caused by degradation
of performance at higher frequencies; the latter remains a task for the user if
precise high-frequency output / input is important for a specific application.

How exactly the corrective gain and offset values are obtained is of no concern
to the Tsunami library - it merely accepts them and corrects its input / output
accordingly; that said, the Calibration sketch does attempt to minimize errors
caused by potential non-linearity by measuring the calibrated values in several
points, calculating a linear regression (the line that fits the measured values
with the least possible amount of total error) and proposing coefficients that
correct for the deviation of that line from the ideal.

The actual calibration API consists of only the following three functions:

----------------------------------------------------------------------------

uint8_t setCalibrationData(CalibratedValue value, float scale, float shift);

Persistently stores and immediately applies calibration coefficients to a single
value where "CalibratedValue" selects the value for which the coefficients are
to be applied, with the following possible values:

CAL_DATA_OFFSET
CAL_DATA_AMPLITUDE
CAL_DATA_MEAN_VALUE
CAL_DATA_PEAK_VALUE
CAL_DATA_CURRENT_VALUE

while "scale" and "shift" are the corrective gain and offset to be applied to
the nominal value, "scale" being dimensionless and "shift" being expressed in
the units of the corrected value: in this case, millivolts for all five values.

Returns "true" if values were successfully stored, "false" otherwise.

Note: the actual generated output waveform will only be affected after the next
offset or amplitude adjustment, NOT as soon as the calibration data is changed.

----------------------------------------------------------------------------

uint8_t getCalibrationData(CalibratedValue value, float *scale, float *shift);

Retrieves (without applying) the current calibration coefficients from EEPROM
persistent storage for a single value selected by "CalibratedValue" which may
take the same values as above; "scale" and "shift" are also defined as above.

Returns "true" if values were actually present and returned, "false" otherwise.

Note: the values returned are the ones currently stored in Tsunami's EEPROM and
NOT whatever is actually being currently loaded / in use.

----------------------------------------------------------------------------

uint8_t useCalibrationData(CalibratedValue value);

Retrieves & applies the current calibration coefficients from EEPROM persistent
storage for a single value or all values as selected by "CalibratedValue" which
may take the same values as above or one of the following additional values:

CAL_DATA_ALL
CAL_DATA_NONE

where the first loads & applies all calibration coefficients at once while the
second unloads them all, returning to an "uncalibrated" state. Multiple calls
for different individual values are perfectly possible, loading any single one
does not unload the others: after loading a value unloading it is only possible
with the collective "none" or by setting "scale" to "1.0" and "shift" to "0.0".

Returns "true" if values were actually present and applied, "false" otherwise.
