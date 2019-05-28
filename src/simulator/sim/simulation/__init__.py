from gym.envs.registration import register

register(
    id='sim-simulation-v0',
    entry_point='simulation.environment:SimRobotEnv'
)