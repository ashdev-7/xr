# PathGuard: walkthrough video (hard limit 3:00)

Recorded with PowerPoint's built-in recorder from `dist/PathGuard_video_deck.pptx` (built by
`tools/make_video_deck.py`); the narration below is also in each slide's speaker notes.

**0:00-0:15** (submission deck slide 1)
> An aircraft is flying with GPS jammed. Its only sensor is an IMU, and the jamming keeps getting worse, until the IMU dies and the aircraft must dead-reckon five more seconds. This is PathGuard, from team ashdev7.

**0:15-0:40** (submission deck slide 2)
> We get a raw EuRoC-format IMU log, degraded with noise, drifting biases, spikes and lost frames, and must output a six-degree-of-freedom pose for every row, in Python, within 8 megabytes and 20 milliseconds per second of data. Integrating the accelerometer twice puts you thousands of metres off.

**0:40-1:10** (submission deck slide 7)
> Here is a real run on a sample flight. It is one standard-library Python file. The log shows what it found: a rotorcraft, from its rotor vibration; the jamming level, read from packet loss; frozen and dead rows; rest periods. Eighty-five seconds of IMU take a third of a second and under three megabytes. One pose per input row, checked by eighteen malformed-input tests.

**1:10-1:45** (submission deck slide 4)
> First, one streaming pass cleans the data and measures the jamming. An error-state Kalman smoother estimates attitude from gyro, gravity and rest periods. Position comes from a second smoother with a motion prior per platform. For drones we use rotor drag: the sideways accelerometer reading is proportional to velocity, so the accelerometer becomes a velocity sensor. Every input row gets one pose.

**1:45-2:15** (submission deck slide 5)
> We validated on real EuRoC and TUM-VI data, degraded with the organizers' own script, on flights held out while we built it. Drones: 2.35 metres ATE against 3.20 for standing still and 18,645 for naive integration. Handheld: 0.52 against 1.10. PathGuard beats standing still on every held-out flight.

**2:15-2:35** (submission deck slide 8)
> This is the drag model at work: body velocity from the accelerometer alone, against ground truth. Every component is backed by an ablation, and we attacked the system with 52 adversarial inputs and checked our scores with the organizers' evo tool.

**2:35-2:50** (submission deck slide 6)
> Physics first, measured at every step, and inside every budget, with a 4x margin on memory and runtime.

**2:50-2:55** (submission deck slide 9)
> PathGuard. Thank you.

