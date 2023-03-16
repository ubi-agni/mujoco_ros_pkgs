# 0.2.0
- Overhaul of sensor publisher model: a SensorConfig struct was added, holding the frame_id, a noise model (mean and standard deviation) for each sensor dimension, and references to ground truth publisher and value+noise publisher.
- A noise model can now be registered via service call.
- If train mode is active (eval_mode is unset), a sensor will publish to two topics: one without noise (sensor_name with a '_GT' suffix) and one which might contain noisy data, if a noise model has been registered.
- Added tests for sensor types (based on message type) and noise models.
