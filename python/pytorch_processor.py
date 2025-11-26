import torch
import numpy as np

from mesh_module import Mesh

def fib(obj: Mesh):
    vertices = obj.get_vertices()
    faces = obj.get_faces()
    print(vertices.shape)
    print(faces)


def process_mesh(mesh):
    
    x = torch.randn(3, 4)
    print(x)
    print("happy")

    vertices_np = mesh.get_vertices()  # 零拷贝到 NumPy
    vertices_tensor = torch.from_numpy(vertices_np)  # 共享内存
    
    print("Vertices Tensor:", vertices_tensor)