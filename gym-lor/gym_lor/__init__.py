from gym.envs.registration import register

register(
    id='lor-v0',
    entry_point='gym_lor.envs:CircleEnv',
)

