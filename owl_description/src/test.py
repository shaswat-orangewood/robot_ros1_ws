import numpy as np

def find_perpendicular_vectors(v1):
    # Step 1: Choose an arbitrary vector A that's not parallel to v1.
    a = np.array([1.0, 0.0, 0.0])  # You can choose any non-parallel vector
    
    # Step 2: Calculate V3 = v1 x a (cross product).
    v3 = np.cross(v1, a)
    
    # Step 3: Calculate V2 = v3 x v1 (cross product).
    v2 = np.cross(v3, v1)
    
    # Step 4: Normalize vectors V2 and V3 (optional).
    v2 /= np.linalg.norm(v2)
    v3 /= np.linalg.norm(v3)
    
    return v2, v3

# Replace this with your known vector V1
known_vector_v1 = np.array([0,-0.3,-0.3])

# Find the other two mutually perpendicular vectors
v2, v3 = find_perpendicular_vectors(known_vector_v1)
rotation_matrix = np.column_stack((known_vector_v1,-v2, -v3))
print(rotation_matrix)
print("Vector V2:", v2)
print("Vector V3:", v3)
