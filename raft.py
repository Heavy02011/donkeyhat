import pygame
import pymunk
import pymunk.pygame_util
import numpy as np
import random

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((1000, 600))
clock = pygame.time.Clock()
draw_options = pymunk.pygame_util.DrawOptions(screen)
screen_height = 600  # Screen height

# Create a pymunk space
space = pymunk.Space()
space.gravity = (0, 981)  # Set gravity to a realistic value (positive for Pygame coordinate system)

# Initialize wave parameters
wave_params = [
    {"amplitude": random.uniform(0.5, 1.5), "wavelength": random.uniform(20, 50), "frequency": random.uniform(0.5, 1.5)}
    for _ in range(5)
]

# Function to create ocean waves
def create_wave(space, x_range, wave_params, phase, offset, width=2):
    wave_points = []
    wave_segments = []
    for x in x_range:
        height = sum(param["amplitude"] * np.sin(2 * np.pi * x / param["wavelength"] + phase * param["frequency"]) for param in wave_params)
        y = screen_height - (offset + height)  # Flip y-coordinate
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        body.position = (x, y)
        shape = pymunk.Segment(body, (0, -width // 2), (0, width // 2), 1)
        shape.friction = 1.0
        space.add(body, shape)
        wave_points.append((x, y))
        wave_segments.append(shape)
    return wave_points, wave_segments

# Create initial wave
x_range = range(0, 1000, 2)
offset = 300
phase = 0
wave_points, wave_segments = create_wave(space, x_range, wave_params, phase, offset)

# Create the stick
stick_width = 100  # Stick width (100 meters)
stick_height = 2  # Stick height (2 meters)
stick_mass = 500  # Stick mass (500 kg)
stick_density = stick_mass / (stick_width * stick_height)  # Density calculation
max_wave_amplitude = max(param["amplitude"] for param in wave_params)
stick_initial_height = offset + max_wave_amplitude + stick_height  # Position stick on top of waves

# Create the stick body and shape
stick_body = pymunk.Body(stick_mass, pymunk.moment_for_box(stick_mass, (stick_width, stick_height)))
stick_body.position = (500, screen_height - stick_initial_height)  # Adjust for flipped y-coordinate
stick_shape = pymunk.Poly.create_box(stick_body, (stick_width, stick_height))
stick_shape.density = stick_density
stick_shape.friction = 0.5
space.add(stick_body, stick_shape)

# Function to calculate forces on the stick
def calculate_forces(stick_body, wave_points):
    forces = []
    positions = [stick_body.position[0] - stick_width / 2, stick_body.position[0], stick_body.position[0] + stick_width / 2]
    for pos in positions:
        wave_height = np.interp(pos, [x for x, y in wave_points], [y for x, y in wave_points])
        wave_height = screen_height - wave_height  # Convert to physics coordinates
        stick_height = screen_height - stick_body.position[1]  # Convert to physics coordinates
        force_magnitude = (stick_height - wave_height) * 981 * stick_density  # Buoyant force = height difference * gravity * density
        forces.append((pos, force_magnitude))
    return forces

# Function to draw arrows representing forces
def draw_forces(screen, forces):
    for pos, force in forces:
        start_pos = (pos, screen_height - stick_body.position[1])
        end_pos = (pos, screen_height - stick_body.position[1] - force / 1000)  # Scale force for visualization
        pygame.draw.line(screen, (255, 0, 0), start_pos, end_pos, 3)
        pygame.draw.polygon(screen, (255, 0, 0), [(end_pos[0], end_pos[1] - 5), (end_pos[0] - 5, end_pos[1] + 5), (end_pos[0] + 5, end_pos[1] + 5)])

# Simulation loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))
    space.debug_draw(draw_options)

    # Update wave positions
    for segment in wave_segments:
        space.remove(segment)  # Remove old wave segments

    phase += 0.1
    wave_points, wave_segments = create_wave(space, x_range, wave_params, phase, offset)

    # Calculate forces on the stick
    forces = calculate_forces(stick_body, wave_points)

    # Draw forces
    draw_forces(screen, forces)

    # Step the physics engine
    dt = 1.0 / 60.0
    space.step(dt)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
