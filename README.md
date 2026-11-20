# MotorXploer Lab 2.1 AutoRun

This MATLAB script runs the FRA271 Lab 2.1 DC motor PWM sweep automatically. It uses the `MotorXploer.slx` Simulink model with an STM32 Nucleo-G474RE board connected over Host Serial at 2 Mbaud.

The model comes from [MINIFI2548/FRA271_LAB2_ACTUATOR_FRAB12](https://github.com/MINIFI2548/FRA271_LAB2_ACTUATOR_FRAB12) and is not included in this repo.

## Usage

1. Put `MotorXploer_Lab21_AutoRun.m` in the same folder as `MotorXploer.slx`.
2. Connect the board and check the serial port in the *Host Serial Setup* block.
3. Edit the `CONFIGURATION` section at the top of the script if needed.
4. Run it from the MATLAB Command Window:

   ```matlab
   MotorXploer_Lab21_AutoRun
   ```

## What it does

- Runs three frequency sets: `[5 50 500 5000 50000]`, `[1 10 100 1000 10000]` and `[2 20 200 2000 20000]` Hz.
- Runs 3 rounds for each frequency.
- In each round, sweeps the duty cycle from 0% to 100% in 10% steps. The PWM value is `round(duty/100*65535)`.
- Logs 10 s per step of:
  - Motor Angular Velocity
  - Load Cell (PA0)
  - Current (PA1)
  - PWM Range (echo)
- After every round, sets PWM to 0 and lets the motor cool down for 300 s.
- Starts the simulation once per round, so the serial port is opened only once per round. Before every stop it sends PWM = 0 to the board for 0.5 s.
- Shows a live status line and a cooldown countdown in the Command Window, with the estimated time remaining.

## Output (`Lab21_Data/`)

- `SetX_FyHz_Rz.mat` holds one finished round. It contains `meta`, `steps` (11 steps, each with the time vector, all 4 signals, set, frequency, round and duty) and `raw` (the full logged data for the round).
- `Lab21_index.mat` is a table of the rounds that have finished.
- `*_PARTIAL.mat` holds the completed steps of a round that was interrupted.
- If you run the script again, it skips rounds that are already saved.

## Safety

If you press Ctrl+C or an error occurs, the script:

1. Sets PWM Range to 0 and sends it to the board.
2. Stops the simulation.
3. Saves any partial data.

If the model can no longer send data, the script writes a PWM = 0 packet directly to the serial port. If that also fails, it tells you to cut motor power.

## Notes

- `MotorXploer/Saturation1` limits Motor Frequency to 0–20000 Hz. By default, frequencies above that limit are **skipped** (`cfg.OverLimitAction`).
- The script turns on signal logging in memory only. It never saves the `.slx` file.
- The full test matrix takes about 4.8 hours, most of it cooldown.
- Do a short test on the real hardware first. For example, set `cfg.DutyPct = [0 50]`, `cfg.NumRounds = 1` and `cfg.Cooldown_s = 5`, then press Ctrl+C during a run and check that the motor stops.
