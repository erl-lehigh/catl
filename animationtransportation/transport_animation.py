from inspect import stack
from turtle import clear, forward
import pygame
import random
import matplotlib as plt
from math import *
import numpy as np

SCREEN_WIDTH = 1800
SCREEN_HEIGHT = 960

BLACK = (0, 0, 0)
WHITE = (250,235,215)
red = (255,0,0)
blue = (0, 0, 255)
green = (0, 255, 0)
verde = (192,255,62)

goal1 = (1040, 750)
goal2 = (100, 500)
goal3 = (1000,  5)

r_pos1 = (5, 5)
r_pos2 = (300, 300)
r_pos3 = (300, 300)  

r_pos = [r_pos1, r_pos2, r_pos3]
goal  = [goal1,  goal2,   goal3]
robot_clasess = [1, 1, 2]
resourcesr_1 = {
                'sand': 2,
                'brick': 0,
                'water': 0,
                'wood': 8
                }

resourcesr_2 = {
                'sand': 2,
                'brick': 0,
                'water': 1,
                'wood': 0
                }

resourcesr_3 = {
                'sand': 2,
                'brick': 1,
                'water': 1,
                'wood': 1
                }
        #resources q2
resources2 = {
            'sand': 2,
            'brick': 0,
            'water': 0,
            'wood': 0
            }

#resources q4
resources4 = {
            'sand': 1,
            'brick': 2,
            'water': 3,
            'wood': 4
            }

#resources q7
resources7 = {
            'sand': 0,
            'brick': 4,
            'water': 0,
            'wood': 0
            }

#resources q8
resources8 = {
            'sand': 0,
            'brick': 0,
            'water': 0,
            'wood': 5
            }

resources_robots = [resourcesr_1, resourcesr_2, resourcesr_3]

class Robot(pygame.sprite.Sprite):

    def __init__(self, r_class, resources, position):
        super().__init__()
        self.r_class = r_class
        self.resources = resources
        self.position = position 

        
        self.image = pygame.image.load("robot2.png").convert()
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()
        self.rect.x = self.position[0]
        self.vel_x = 0
        self.vel_y = 0
        self.vel_yaw = 0
        self.rect.y = self.position[1]


    def update(self):
        self.rect.x += self.vel_x 
        self.rect.y += self.vel_y



    def move(self, pd_x, pd_y): 

        dt = 0.001
        self.vel_x = pd_x * dt
        self.vel_y = pd_y * dt

    
    def control(self, desired_pos, desired_vel):
        kp, kd = 10, 15

        error_pos_x = desired_pos[0] - self.rect.x
        error_vel_x = desired_vel[0] - self.vel_x

        error_pos_y = desired_pos[1] - self.rect.y 
        error_vel_y = desired_vel[1] - self.vel_y

        pd_x = (kp * error_pos_x + kd * error_vel_x)
        pd_y = (kp * error_pos_y + kd * error_vel_y)

        return pd_x, pd_y 
    

class Node(pygame.sprite.Sprite):

    def __init__(self, node_class, node_pos, resources, capabilities):
        super().__init__()
        
        self.resources = resources
        self.capabilities = capabilities
        self.node_class = node_class
        self.node_pos = node_pos


        self.image = pygame.image.load(node_class).convert()
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()
        self.rect.x = node_pos[0]
        self.rect.y = node_pos[1]
        pass

            

class Game(object):

    def __init__(self, n_robots=3):
        self.score = 0
        self.nodes_list = pygame.sprite.Group()
        self.all_sprites_list = pygame.sprite.Group()
        nodes = 8
        self.n_robots = n_robots
        self.robot = []
        nodes = []

        nodes.append(Node("cons_node.png", (5,5), None, 'capabilities'))
        nodes.append(Node("node2.png", (375,160), resources2, 'capabilities'))
        nodes.append(Node("cons_node.png", (750,5), None, 'capabilities'))
        nodes.append(Node("node4.png", (10, 540), resources4, 'capabilities'))
        nodes.append(Node("cons_node.png", (500,580), None, 'capabilities'))
        nodes.append(Node("cons_node.png", (1100,250), None, 'capabilities'))
        nodes.append(Node("node7.png", (920, 580), resources7, 'capabilities'))
        nodes.append(Node("node8.png", (1440,30), resources8, 'capabilities'))
        nodes.append(Node("cons_node.png", (1350,580), None, 'capabilities'))

        for node in nodes:
            self.nodes_list.add(node)
            self.all_sprites_list.add(node)
        
        for i in range(self.n_robots):
            self.robot.append(Robot(robot_clasess[i], resources_robots[i], r_pos[i]))
            self.all_sprites_list.add(self.robot[i])


    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

        return False

    def run_logic(self):
        for i in range(self.n_robots):
            f, t = self.robot[i].control(goal[i], (0,0))
            self.robot[i].move(f,t)

        self.all_sprites_list.update()


    def display(self, screen):
        
        screen.fill(WHITE)
        font = pygame.font.SysFont("Raleway Bold", 60)
        font3 = pygame.font.SysFont("Raleway Bold", 110)
        font2 = pygame.font.SysFont("Raleway Bold", 50) # Fuente
        text = font.render("q1", True, BLACK)
        text2 = font.render("q2", True, BLACK)
        text2_resources = font3.render(str(resources2['sand']), True, verde)
        text3 = font.render("q3", True, BLACK)
        text4 = font.render("q4", True, BLACK)
        t4_re1 = font3.render(str(resources4['sand']), True, verde)
        t4_re2 = font3.render(str(resources4['water']), True, verde)
        t4_re3 = font3.render(str(resources4['brick']), True, verde)
        t4_re4 = font3.render(str(resources4['wood']), True, verde)
        text5 = font.render("q5", True, BLACK)
        text6 = font.render("q6", True, BLACK)
        text7 = font.render("q7", True, BLACK)
        text7_re = font3.render(str(resources7['brick']), True, verde)
        text8 = font.render("q8", True, BLACK)
        text8_re = font3.render(str(resources7['wood']), True, verde)
        text9 = font.render("q9", True, BLACK) 
        screen.blit(text, [5, 5])
        screen.blit(text2, [375, 160])
        
        screen.blit(text3, [750, 5]) 
        screen.blit(text4, [10, 540]) 
        screen.blit(text5, [450, 593]) 
        screen.blit(text6, [1100, 250]) 
        screen.blit(text7, [920, 580]) 
        screen.blit(text8, [1440, 30]) 
        screen.blit(text9, [1350, 580])  

        pygame.draw.line(screen, BLACK, [5+180, 5+180], [375+180, 160+180], 10)
        pygame.draw.line(screen, BLACK, [750+180, 5+180], [1100+180, 250+180], 10)
        pygame.draw.line(screen, BLACK, [5+180, 5+180], [500+180, 580+180], 10)
        pygame.draw.line(screen, BLACK, [5+180, 5+180], [10+180, 540+180], 10)    
        pygame.draw.line(screen, BLACK, [375+180, 160+180], [750+180, 5+180], 10)
        pygame.draw.line(screen, BLACK, [375+180, 160+180], [500+180, 580+180], 10)
        pygame.draw.line(screen, BLACK, [750+180, 5+180], [1440+180, 30+180], 10)
        pygame.draw.line(screen, BLACK, [375+180, 160+180], [500+180, 580+180], 10)
        pygame.draw.line(screen, BLACK, [1430+180, 30+180], [750+180, 5+180], 10)
        pygame.draw.line(screen, BLACK, [1430+180, 30+180], [1100+180, 250+180], 10)
        pygame.draw.line(screen, BLACK, [1430+180, 30+180], [1350+180, 580+180], 10)
        pygame.draw.line(screen, BLACK, [1350+180, 580+180], [1100+180, 250+180], 10)
        pygame.draw.line(screen, BLACK, [1350+180, 580+180], [920+180, 580+180], 10)
        pygame.draw.line(screen, BLACK, [920+180, 580+180], [1100+180, 250+180], 10)
        pygame.draw.line(screen, BLACK, [920+180, 580+180], [500+180, 580+180], 10)
        pygame.draw.line(screen, BLACK, [10+180, 540+180], [500+180, 580+180], 10)

        
        
        
        sand  = pygame.image.load('sand.png').convert()
        sand = pygame.transform.scale(sand, (40, 40))
        sand.set_colorkey(BLACK)
        
        brick = pygame.image.load('brick.png').convert()
        brick = pygame.transform.scale(brick, (40, 40))
        brick.set_colorkey(BLACK)

        water = pygame.image.load('water.png').convert()
        water = pygame.transform.scale(water, (40, 40))
        water.set_colorkey(BLACK)

        wood  = pygame.image.load('wood.png').convert()
        wood = pygame.transform.scale(wood, (35, 35))
        wood.set_colorkey(BLACK)

        self.all_sprites_list.draw(screen)

        screen.blit(text2_resources, [375+160, 160+134])

        screen.blit(t4_re1, [10+235, 540+220])
        screen.blit(t4_re2, [10+114, 540+54])
        screen.blit(t4_re3, [10+250, 540+95])
        screen.blit(t4_re4, [10+90, 540+178])

        screen.blit(text7_re, [920+170, 580+140])

        screen.blit(text8_re, [1440+170,30+140])

        for i in range(self.n_robots):
            x, y = self.robot[i].rect.x, self.robot[i].rect.y

            if self.robot[i].r_class == 1:
                pygame.draw.circle(screen, red, (x+75, y+67), 25)
            if self.robot[i].r_class == 2:
                pygame.draw.circle(screen, blue, (x+75, y+67), 25)
            
            if self.robot[i].resources['sand'] > 0:
                screen.blit(sand, [x+30, y+20])
                res_amount = font2.render(str(self.robot[i].resources['sand']), True, verde) 
                screen.blit(res_amount, [x+40, y+24])

            if self.robot[i].resources['brick'] > 0:
                screen.blit(brick, [x+30, y+76])
                res_amount = font2.render(str(self.robot[i].resources['brick']), True, verde) 
                screen.blit(res_amount, [x+40, y+80])

            if self.robot[i].resources['water'] > 0: 
                screen.blit(water, [x+82, y+20])
                res_amount = font2.render(str(self.robot[i].resources['water']), True, verde) 
                screen.blit(res_amount, [x+93, y+26])

            if self.robot[i].resources['wood'] > 0: 
                screen.blit(wood, [x+87, y+75])
                res_amount = font2.render(str(self.robot[i].resources['wood']), True, verde) 
                screen.blit(res_amount, [x+95, y+78]) 

        pygame.display.flip()


def main():
    pygame.init()

    screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

    done = False
    clock = pygame.time.Clock()

    game = Game()

    while not done:
        done = game.process_events()
        game.run_logic()
        game.display(screen)
        clock.tick(20)
    pygame.quit()


if __name__ == "__main__":
    main()
