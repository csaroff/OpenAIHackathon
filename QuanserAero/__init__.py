from gym.envs.registration import register

register(
    id='Aero-v0',
    entry_point='aero_env:AeroEnv',
    )