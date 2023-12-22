'''
@ name: Austin
@ description: 
@ Date: 2023-10-17 13:31:35
@ LastEditDate: 
'''
import random
import math
import calibrateByPad.PSO_function as func

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
            
            if abs(particle.fitness) < abs(particle.best_fitness):
                particle.best_position = particle.position.copy()
                particle.best_fitness = particle.fitness
                if abs(particle.best_fitness) < abs(self.global_best_particle.best_fitness):
                    global_best_particle_index = i
        if global_best_particle_index != -1:
            self.global_best_particle.__dict__ = self.particle_set[global_best_particle_index].__dict__.copy()

    def randomly_initial(self):
        '''初始化粒子群, 并求得目前最优解
        '''
        # 最优解坐标维-1
        global_best_particle_index = -1
        particle = self.particle_set[0]

        for j in range(self.dimension):
            # 初始化第j维的位置、最佳位置和速度
            temp_val = self.position_min_value[j] + self.rand_0_1() * (self.position_max_value[j] - self.position_min_value[j])
            particle.position[j] = temp_val
            particle.best_position[j] = temp_val
            particle.velocity[j] = self.rand_0_1()

        # 速度模长 - 各维度速度平方和再开方
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
            # 不是很理解对于速度的这几步操作
            velocity_mod = math.sqrt(sum([v ** 2 for v in particle.velocity]))
            v_mod = self.rand_0_1() * self.max_speed
            for j in range(self.dimension):
                particle.velocity[j] *= (v_mod / velocity_mod)
            particle.fitness = self.obj_function(particle)
            particle.best_fitness = particle.fitness

            # 谁的绝对值小,谁就是最优解
            if abs(particle.best_fitness) < abs(self.global_best_particle.best_fitness):
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
            v_mod_1 = 0
            v_mod_2 = 0
            r1 = self.rand_0_1()
            r2 = self.rand_0_1()
            for j in range(self.dimension):
                particle.velocity[j] += self.global_guide_coe * r1 * (self.global_best_particle.best_position[j] - particle.position[j])
                particle.velocity[j] += self.local_guide_coe * r2 * (particle.best_position[j] - particle.position[j])
                if j>3:
                    v_mod_1 += particle.velocity[j] ** 2

            # 添加约束条件 
            # (x^2 + y^2 + z^2 + w^2)-1<c
            # while (True):
            sum = 0
            #     v_mod_2 = 0

            for j in range(4):
                particle.velocity[j] += self.global_guide_coe * r1 * (self.global_best_particle.best_position[j] - particle.position[j])
                particle.velocity[j] += self.local_guide_coe * r2 * (particle.best_position[j] - particle.position[j])
                v_mod_2 += particle.velocity[j] ** 2

                sum += (particle.velocity[j]+particle.position[j]) ** 2
            
            # if abs(sum-1) > 1:
            #     continue
            
            # print(sum)
            
            v_mod = math.sqrt(v_mod_1 + v_mod_2)
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

    def find_zero(self, times, disturbance_rate=0.2, disturbance_velocity_coe=0.05):
        '''求解最大值(最优解)

        Args:
            times (迭代次数): _description_
            disturbance_rate (float, optional): 扰动率，用于控制粒子的扰动程度. Defaults to 0.2.
            disturbance_velocity_coe (float, optional): 扰动速率系数，用于控制粒子扰动时的速度. Defaults to 0.05.

        Returns:
            Particle: 最大值(最优解)
        '''
        self.randomly_initial()
        for i in range(times):
            self.update(disturbance_rate, disturbance_velocity_coe)
        best_particle = Particle(self.dimension)
        best_particle.__dict__ = self.global_best_particle.__dict__.copy()
        return best_particle



# 每个参数的最小值
position_min_value = [-2, -2, -2, -0.8, -100, 100, 1125]
# 每个参数的最大值
position_max_value = [2, 2, 2, 0.8, 100, 300, 1200]
# 参数个数，也就是问题的维度。
dimension = 7
# 粒子数，也就是解的数量
particle_count = 500
# 全局引导系数，用于控制粒子向全局最优解靠近的程度。
global_guide_coe = 2
# 局部引导系数，用于控制粒子向局部最优解靠近的程度。
local_guide_coe = 2
# 粒子的最大速度。
max_speed = 1
# 创建PSO对象
pso = PSO(func.pso_function, position_min_value, position_max_value, dimension, particle_count, global_guide_coe, local_guide_coe, max_speed)
# 求解最优解(最大值)
best_particle = pso.find_zero(100)
print(best_particle.position, best_particle.best_fitness)


