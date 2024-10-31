'''
task1 = find_best_control_method(robot, env)
control_method = task1.results.control_method // `velocity`/`position`
task2 = find_best_control_parameters(control_method, robot)
control_parameters = task2.results.control_parameters // {P = 1, D = 1.2, I = 0.8} or {P = 1, D = 1.1}
downsample_ratio = task2.results.downsample_ratio // this is to guarantee stability
controller = get_controller(‘velocity’, ‘pid’, P=1, D=1.2, I=0.9)
while True:
   top_view_obs = env.top_view_sensor.fetch_observations(…)
   imu_obs = robot.imu_sensor.fetch_observation(…)
   if time % downsample_ratio then: // in rare cases…
       robot_position = estimator.sensor_fusion(top_view_obs, imu_obs) 
       action = controller.step(robot_position) 
       robot.perform_action(action) 
    engine.step(timestamp=…) / time.sleep
    if stop_condition_is_met(self, env, robot):
       break
'''