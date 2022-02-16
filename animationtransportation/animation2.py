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

goal1 =(1040, 750)
goal2 = (100, 500)

class Robot(pygame.sprite.Sprite):

    def __init__(self, r_class, resources, position):
        super().__init__()
        
        
        self.r_class = r_class
        self.resources = resources
        self.position = position 

        
        self.image = pygame.image.load("robot2.png").convert()
        # self.image = pygame.transform.rotate(self.image, position[2])
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()
        self.rect.x = self.position[0]
        self.vel_x = 0
        self.vel_y = 0
        self.vel_yaw = 0
        self.rect.y = self.position[1]
        # self.orientation = self.position[2]

    def update(self):
        self.rect.x += self.vel_x 
        self.rect.y += self.vel_y
        self.orientation = atan2(self.vel_y, self.vel_x) * 180/pi
        print(self.orientation)


    def move(self, pd_x, pd_y): #forward, turn):

        dt = 0.001
        self.vel_x = pd_x * dt
        self.vel_y = pd_y * dt


        # if (self.rect.y <= 10 or self.rect.y >= 750 or 
        #     self.rect.x <= 0  or self.rect.x >= 1750):

        #     # self.vel_yaw = 0
        #     self.vel_x   = 0
        #     self.vel_y   = 0
    
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

    def __init__(self):
        self.score = 0
        self.nodes_list = pygame.sprite.Group()
        self.all_sprites_list = pygame.sprite.Group()
        nodes = 8
                  
        nodes = []

        #resources q2
        resources2 = {
                    'sand': 2,
                    'brick': 0,
                    'water': 0,
                    'wood': 0
                    }

        #resources q4
        resources4 = {
                    'sand': 2,
                    'brick': 2,
                    'water': 2,
                    'wood': 2
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
        
        r_pos = (5, 5)
        r_pos2 = (300, 300)       
        self.robot = Robot(1, 1, r_pos)
        self.robot2 = Robot(1,1, r_pos2)
        self.all_sprites_list.add(self.robot)
        self.all_sprites_list.add(self.robot2)

    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

        return False

    def run_logic(self):
        f, t = self.robot.control(goal1, (0,0))
        f2, t2 = self.robot2.control(goal2, (0,0))
        self.robot.move(f,t)
        self.robot2.move(f2, t2)
        self.all_sprites_list.update()


    def display(self, screen):
        screen.fill(WHITE)
        font = pygame.font.SysFont("serif", 40) # Fuente
        text = font.render("q1", True, BLACK)
        text2 = font.render("q2", True, BLACK)
        text3 = font.render("q3", True, BLACK)
        text4 = font.render("q4", True, BLACK)
        text5 = font.render("q5", True, BLACK)
        text6 = font.render("q6", True, BLACK)
        text7 = font.render("q7", True, BLACK)
        text8 = font.render("q8", True, BLACK)
        text9 = font.render("q9", True, BLACK) 
        screen.blit(text, [5, 5])
        screen.blit(text2, [375, 160]) 
        screen.blit(text3, [750, 5]) 
        screen.blit(text4, [10, 540]) 
        screen.blit(text5, [500, 580]) 
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
        
        x,y,theta = self.robot.rect.x, self.robot.rect.y, self.robot.orientation
        x2,y2,theta2 = self.robot2.rect.x, self.robot2.rect.y, self.robot2.orientation

        self.all_sprites_list.draw(screen)
        resourcesr_1 = {
                    'sand': 2,
                    'brick': 0,
                    'water': 0,
                    'wood': 0
                    }
        pygame.draw.circle(screen, red, (x+75, y+67), 25)
        screen.blit(sand, [x+30, y+20])
        screen.blit(brick, [x+30, y+76]) 
        screen.blit(water, [x+82, y+20]) 
        screen.blit(wood, [x+87, y+75]) 
        

        pygame.draw.circle(screen, blue, (x2+75, y2+67), 25)
        screen.blit(sand, [x2+30, y2+20])
        screen.blit(brick, [x2+30, y2+76]) 
        screen.blit(water, [x2+82, y2+20]) 
        screen.blit(wood, [x2+87, y2+75])
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
