def determine_desired_pose(self, desired_angle):
    distance_guard = 2 * np.sqrt(2)
    desired_distance = 2
    
    # Lista para almacenar las correcciones de cada vecino
    corrections = []
    
    # Iterar sobre todos los vecinos
    for neighbor_position in self.all_neighbor_positions:
        corDiff = np.subtract(self.cord[:2], neighbor_position[:2])
        distance_to_neighbor = np.linalg.norm(corDiff)
        
        # Aplicar la lógica de corrección según la distancia al vecino
        if distance_to_neighbor <= distance_guard / 2:
            aux = (distance_guard - distance_to_neighbor) / distance_to_neighbor
            correction = np.multiply(corDiff, aux)
            corrections.append(correction)
    
    # Si hay correcciones, combinarlas
    if corrections:
        total_correction = np.sum(corrections, axis=0)
        new_pose = np.add(self.cord[:2], total_correction)
        return np.hstack((new_pose, self.cord[2:4]))
    else:
        # Si no hay correcciones, proceder con la lógica del líder
        x, y, z, yaw = self.leader_position
        desired_x = x + desired_distance * np.cos(np.radians(desired_angle))
        desired_y = y + desired_distance * np.sin(np.radians(desired_angle))
        return [desired_x, desired_y, z - 0.4, yaw]


# Si tienes múltiples vecinos, puedes adaptar el código para manejar cada vecino por separado y luego combinar las correcciones necesarias. Aquí te dejo una idea de cómo podrías hacerlo:

#     Para cada vecino, calcula la diferencia de posición y la distancia al igual que lo estás haciendo actualmente.
#     Evalúa cada distancia y aplica la lógica de corrección según la distancia al vecino más cercano. Si estás muy cerca de cualquier vecino, ajusta tu posición para alejarte de ese vecino siguiendo la misma lógica que ya has implementado.
#     Si estás lo suficientemente lejos de todos los vecinos, puedes proceder con la lógica actual para mantener una distancia y ángulo constante respecto al líder.