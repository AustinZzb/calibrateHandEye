'''
@ name: Austin
@ description: 使用现有数据利用PSO求解手眼矩阵, 优化效果微乎其微
@ Date: 2023-09-27 10:07:23
@ LastEditDate: 
'''
import random
import math

class Particle:
    def __init__(self, dimension):
        self.dimension = dimension
        self.position = [0] * dimension
        self.velocity = [0] * dimension
        self.best_position = [0] * dimension
        self.fitness = 0
        self.best_fitness = 0

class PSO:
    def __init__(self, obj_function, position_min_value, position_max_value, dimension, particle_count, global_guide_coe=2, local_guide_coe=2, max_speed=1):
        self.obj_function = obj_function
        self.position_min_value = position_min_value
        self.position_max_value = position_max_value
        self.dimension = dimension
        self.particle_count = particle_count
        self.global_guide_coe = global_guide_coe
        self.local_guide_coe = local_guide_coe
        self.max_speed = max_speed
        self.particle_set = [Particle(self.dimension) for i in range(self.particle_count)]
        self.global_best_particle = Particle(self.dimension)

    def rand_0_1(self):
        return random.uniform(0, 1)

    def refresh(self):
        global_best_particle_index = -1
        for i in range(self.particle_count):
            particle = self.particle_set[i]
            particle.fitness = self.obj_function(particle)
            if particle.fitness > particle.best_fitness:
                particle.best_position = particle.position.copy()
                particle.best_fitness = particle.fitness
                if particle.best_fitness > self.global_best_particle.best_fitness:
                    global_best_particle_index = i
        if global_best_particle_index != -1:
            self.global_best_particle.__dict__ = self.particle_set[global_best_particle_index].__dict__.copy()

    def randomly_initial(self):
        global_best_particle_index = -1
        particle = self.particle_set[0]
        for j in range(self.dimension):
            temp_val = self.position_min_value[j] + self.rand_0_1() * (self.position_max_value[j] - self.position_min_value[j])
            particle.position[j] = temp_val
            particle.best_position[j] = temp_val
            particle.velocity[j] = self.rand_0_1()
        velocity_mod = math.sqrt(sum([v ** 2 for v in particle.velocity]))
        v_mod = self.rand_0_1() * self.max_speed
        for j in range(self.dimension):
            particle.velocity[j] *= (v_mod / velocity_mod)
        particle.fitness = self.obj_function(particle)
        particle.best_fitness = particle.fitness
        self.global_best_particle.__dict__ = particle.__dict__.copy()
        
        for i in range(1, self.particle_count):
            particle = self.particle_set[i]
            for j in range(self.dimension):
                temp_val = self.position_min_value[j] + self.rand_0_1() * (self.position_max_value[j] - self.position_min_value[j])
                particle.position[j] = temp_val
                particle.best_position[j] = temp_val
                particle.velocity[j] = self.rand_0_1()
            velocity_mod = math.sqrt(sum([v ** 2 for v in particle.velocity]))
            v_mod = self.rand_0_1() * self.max_speed
            for j in range(self.dimension):
                particle.velocity[j] *= (v_mod / velocity_mod)
            particle.fitness = self.obj_function(particle)
            particle.best_fitness = particle.fitness
            if particle.best_fitness > self.global_best_particle.best_fitness:
                global_best_particle_index = i
        
        if global_best_particle_index != -1:
            self.global_best_particle.__dict__ = self.particle_set[global_best_particle_index].__dict__.copy()

    def disturbance(self, particle, relative_velocity_rate=0.05):
        disturbance_velocity = [0] * self.dimension
        disturbance_velocity_mod = relative_velocity_rate * self.max_speed * self.rand_0_1()
        v_mod = 0
        for i in range(self.dimension):
            disturbance_velocity[i] = self.rand_0_1()
            v_mod += disturbance_velocity[i] ** 2
        v_mod = math.sqrt(v_mod)
        for i in range(self.dimension):
            disturbance_velocity[i] *= (disturbance_velocity_mod / v_mod)
        v_mod = 0
        for i in range(self.dimension):
            particle.velocity[i] += disturbance_velocity[i]
            v_mod += particle.velocity[i] ** 2
        v_mod = math.sqrt(v_mod)
        if v_mod > self.max_speed:
            for i in range(self.dimension):
                particle.velocity[i] *= (self.max_speed / v_mod)

    def update(self, disturbance_rate=0.2, disturbance_velocity_coe=0.05):
        for i in range(self.particle_count):
            particle = self.particle_set[i]
            v_mod = 0
            r1 = self.rand_0_1()
            r2 = self.rand_0_1()
            for j in range(self.dimension):
                particle.velocity[j] += self.global_guide_coe * r1 * (self.global_best_particle.best_position[j] - particle.position[j])
                particle.velocity[j] += self.local_guide_coe * r2 * (particle.best_position[j] - particle.position[j])
                v_mod += particle.velocity[j] ** 2
            v_mod = math.sqrt(v_mod)
            if v_mod > self.max_speed:
                for j in range(self.dimension):
                    particle.velocity[j] *= (self.max_speed / v_mod)
            if self.rand_0_1() < disturbance_rate:
                self.disturbance(particle, disturbance_velocity_coe)
            for j in range(self.dimension):
                particle.position[j] += particle.velocity[j]
                if particle.position[j] < self.position_min_value[j]:
                    particle.position[j] = self.position_min_value[j]
                elif particle.position[j] > self.position_max_value[j]:
                    particle.position[j] = self.position_max_value[j]
        self.refresh()

    def find_max(self, times, disturbance_rate=0.2, disturbance_velocity_coe=0.05):
        self.randomly_initial()
        for i in range(times):
            self.update(disturbance_rate, disturbance_velocity_coe)
        best_particle = Particle(self.dimension)
        best_particle.__dict__ = self.global_best_particle.__dict__.copy()
        return best_particle
    
def quaternion():
    pass

def test_function(particle):
    x, y = particle.position
    return -x ** 3 - y ** 4

position_min_value = [-10, -10]
position_max_value = [10, 10]
dimension = 2
particle_count = 100
global_guide_coe = 2
local_guide_coe = 2
max_speed = 1
pso = PSO(test_function, position_min_value, position_max_value, dimension, particle_count, global_guide_coe, local_guide_coe, max_speed)
best_particle = pso.find_max(100)
print(best_particle.position, best_particle.best_fitness)


# 绘制3d图
# import matplotlib.pyplot as plt
# import numpy as np

# x = np.linspace(-10, 10, 100)
# y = np.linspace(-10, 10, 100)

# X, Y = np.meshgrid(x, y)

# Z = -X ** 3 - Y ** 4

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot_surface(X, Y, Z)
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.show()