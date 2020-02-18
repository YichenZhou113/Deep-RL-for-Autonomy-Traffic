class IDMDriver(object):
    def __init__(self,
                 t_init,
                 x_init,
                 vel_init,
                 length=0.165,
                 v0=3,
                 T=0.5,
                 a=0.2,
                 b=3,
                 delta=4,
                 s0=0.05,
                 dt=0.1):
        self.length = length
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        self.dt = dt
        self.t = t_init
        self.x = x_init
        self.vel = vel_init

    def step(self, leader):
        sstar = self.__calc_sstar(self.vel, leader.vel)
        accel = self.a * \
            (1 -
             (self.vel / self.v0) ** self.delta -
             (sstar / ((leader.x - self.x - self.length) % 8.17)) ** 2)
        return accel

    def update(self, t, x, vel):
        self.t = t
        self.x = x 
        self.vel = vel

    def __calc_sstar(self, vel, leader_vel):
        sstar = self.s0 + vel * self.T + \
                vel * (vel - leader_vel) / (2 * np.sqrt(self.a * self.b))
        return sstar
