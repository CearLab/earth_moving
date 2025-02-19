import pygame
import math

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()


# Agent class
class Agent:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color
        self.actions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Right, Down, Left, Up

    def move(self, action):
        dx, dy = action
        self.x += dx
        self.y += dy

    def draw(self):
        pygame.draw.circle(screen, self.color, (self.x * 20, self.y * 20), 10)


# Simulation loop
agents = [Agent(5, 5, (255, 0, 0)), Agent(10, 10, (0, 0, 255))]
running = True

while running:
    screen.fill((255, 255, 255))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update and draw each agent
    for agent in agents:
        action = agent.actions[0]  # Choose action (placeholder for more complex logic)
        agent.move(action)
        agent.draw()

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
