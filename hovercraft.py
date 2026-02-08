import pygame
import numpy as np
import random

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((1000, 600))
clock = pygame.time.Clock()
screen_height = 600  # Screen height

# Colors
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# Wave parameters
wave_params = [
    {"amplitude": random.uniform(0.5, 1.5), "wavelength": random.uniform(20, 50), "frequency": random.uniform(0.5, 1.5)}
    for _ in range(5)
]

# Hovercraft parameters
hovercraft_width = 200
hovercraft_mass = 1000  # kg
skirt_length = 20  # Skirt length
gravity = 9.81  # m/s^2
air_density = 1.225  # kg/m^3
fan_flow_rate = 10  # m^3/s, constant air input from the fan
target_hover_height = 5  # Target hover height above water

# Function to create wave points
def create_wave(x_range, wave_params, phase, offset):
    wave_points = []
    for x in x_range:
        height = sum(param["amplitude"] * np.sin(2 * np.pi * x / param["wavelength"] + phase * param["frequency"]) for param in wave_params)
        y = screen_height - (offset + height)
        wave_points.append((x, y))
    return wave_points

# Function to calculate lift force
def calculate_lift_force(pressure, width, effective_height):
    return pressure * width * effective_height

# Function to calculate air leakage
def calculate_air_leakage(pressure, gap, width):
    if gap <= 0:
        return 0
    # Leakage increases rapidly with gap size
    leakage_coefficient = 0.1 * np.sqrt(gap)
    return leakage_coefficient * pressure * width

# Main loop
running = True
phase = 0
hovercraft_y = 280  # Initial y-position of the hovercraft deck
pressure = 0  # Start with no pressure
dt = 1/60  # Time step

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))  # White background

    # Create and draw wave
    x_range = range(0, 1000, 2)
    offset = 300
    wave_points = create_wave(x_range, wave_params, phase, offset)
    pygame.draw.lines(screen, GREEN, False, wave_points, 2)

    # Calculate average wave height under hovercraft
    deck_left = 500 - hovercraft_width // 2
    deck_right = 500 + hovercraft_width // 2
    wave_heights = [y for x, y in wave_points if deck_left <= x <= deck_right]
    avg_wave_height = sum(wave_heights) / len(wave_heights)

    # Calculate gap between skirt and water
    gap = max(0, (hovercraft_y + skirt_length) - avg_wave_height)

    # Calculate effective height (for lift force calculation)
    effective_height = min(skirt_length, skirt_length - gap)

    # Calculate lift force
    lift_force = calculate_lift_force(pressure, hovercraft_width, effective_height)

    # Calculate air leakage
    leakage = calculate_air_leakage(pressure, gap, hovercraft_width)

    # Adjust hovercraft height
    net_force = lift_force - hovercraft_mass * gravity
    acceleration = net_force / hovercraft_mass
    hovercraft_y += acceleration * dt * 5  # Reduced scaling factor for more gradual movement

    # Adjust pressure (increase due to fan, decrease due to leakage)
    volume = hovercraft_width * effective_height
    if volume > 0:
        pressure_change = (fan_flow_rate - leakage) / volume
        pressure += pressure_change * dt
        pressure = max(pressure, 0)  # Ensure pressure doesn't go negative

    # Debug output
    print(f"Inflow air volume rate: {fan_flow_rate:.2f} m³/s")
    print(f"Leakage volume rate: {leakage:.2f} m³/s")
    print(f"Net internal pressure: {pressure:.2f} Pa")
    print(f"Gap: {gap:.2f} pixels")
    print(f"Hover Height: {(avg_wave_height - hovercraft_y):.2f} pixels")
    print("--------------------")

    # Draw hovercraft
    pygame.draw.line(screen, BLUE, (deck_left, hovercraft_y), (deck_right, hovercraft_y), 2)
    pygame.draw.line(screen, BLUE, (deck_left, hovercraft_y), (deck_left, hovercraft_y + skirt_length), 2)
    pygame.draw.line(screen, BLUE, (deck_right, hovercraft_y), (deck_right, hovercraft_y + skirt_length), 2)

    # Draw fan input (blue arrow in the middle)
    fan_x = (deck_left + deck_right) // 2
    pygame.draw.line(screen, BLUE, (fan_x, hovercraft_y), (fan_x, hovercraft_y + skirt_length), 2)
    pygame.draw.polygon(screen, BLUE, [(fan_x - 5, hovercraft_y + skirt_length),
                                       (fan_x + 5, hovercraft_y + skirt_length),
                                       (fan_x, hovercraft_y + skirt_length + 10)])

    # Draw air leakage (red arrows at the sides)
    if gap > 0:
        for x in [deck_left, deck_right]:
            pygame.draw.line(screen, RED, (x, hovercraft_y + skirt_length), (x + np.sign(x - fan_x) * 10, hovercraft_y + skirt_length), 2)
            pygame.draw.polygon(screen, RED, [(x + np.sign(x - fan_x) * 10, hovercraft_y + skirt_length - 5),
                                              (x + np.sign(x - fan_x) * 10, hovercraft_y + skirt_length + 5),
                                              (x + np.sign(x - fan_x) * 20, hovercraft_y + skirt_length)])

    # Display pressure, gap, and hover height
    font = pygame.font.Font(None, 36)
    pressure_text = font.render(f"Pressure: {pressure:.2f} Pa", True, RED)
    gap_text = font.render(f"Gap: {gap:.2f} pixels", True, RED)
    hover_height_text = font.render(f"Hover Height: {(avg_wave_height - hovercraft_y):.2f} pixels", True, RED)
    screen.blit(pressure_text, (10, 10))
    screen.blit(gap_text, (10, 50))
    screen.blit(hover_height_text, (10, 90))

    pygame.display.flip()
    clock.tick(60)

    phase += 0.1

pygame.quit()