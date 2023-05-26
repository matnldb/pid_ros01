import numpy as np

# Función objetivo (error cuadrático medio)
def objective_function(params):
    kp, kd, ki = params
    # Aquí debes ejecutar tu controlador PID con los parámetros dados y obtener el error cuadrático medio
    # Retorna el valor del error cuadrático medio como resultado

# Función de sintonización automática del PID utilizando PSO
def tune_pid_with_pso():
    # Parámetros del PSO
    num_particles = 20
    num_iterations = 50
    num_dimensions = 3  # kp, kd, ki

    # Rangos de los parámetros
    param_ranges = np.array([[0.0, 1.0], [0.0, 1.0], [0.0, 1.0]])  # Ejemplo de rangos, ajústalos según tus necesidades

    # Mejor posición global encontrada
    best_global_position = np.zeros(num_dimensions)
    best_global_error = float('inf')

    # Inicialización de partículas
    particles = np.random.uniform(low=param_ranges[:, 0], high=param_ranges[:, 1], size=(num_particles, num_dimensions))
    velocities = np.zeros((num_particles, num_dimensions))
    particle_best_positions = particles.copy()
    particle_best_errors = np.full(num_particles, float('inf'))

    # Iteraciones del PSO
    for iteration in range(num_iterations):
        # Evaluación de la función objetivo para cada partícula
        errors = np.array([objective_function(p) for p in particles])

        # Actualización de la mejor posición de cada partícula
        mask = errors < particle_best_errors
        particle_best_positions[mask] = particles[mask]
        particle_best_errors[mask] = errors[mask]

        # Actualización de la mejor posición global
        best_particle = np.argmin(particle_best_errors)
        if particle_best_errors[best_particle] < best_global_error:
            best_global_position = particle_best_positions[best_particle].copy()
            best_global_error = particle_best_errors[best_particle]

        # Actualización de las velocidades y posiciones de las partículas
        inertia_weight = 0.7  # Factor de peso inercial
        cognitive_weight = 1.4  # Factor de peso cognitivo (influencia del mejor propio)
        social_weight = 1.4  # Factor de peso social (influencia del mejor global)

        r1 = np.random.rand(num_particles, num_dimensions)
        r2 = np.random.rand(num_particles, num_dimensions)

        velocities = (inertia_weight * velocities +
                      cognitive_weight * r1 * (particle_best_positions - particles) +
                      social_weight * r2 * (best_global_position - particles))

        particles += velocities

        # Limitar las posiciones dentro de los rangos permitidos
        particles = np.clip(particles, param_ranges[:, 0], param_ranges[:, 1])

    # Mejor conjunto de parámetros encontrados
    best_kp, best_kd, best_ki = best_global_position

    # Devolver los mejores parámetros encontrados
    return best_kp, best_kd, best_ki


# Uso de la función de sintonización
