import numpy as np
from pytorch_arap.arap import ARAP_from_meshes, add_one_ring_neighbours,add_n_ring_neighbours, create_ARAP_meshes
from pytorch_arap.arap import compute_energy as arap_loss
from pytorch3d.io import load_objs_as_meshes, save_obj
import os
import torch
from matplotlib import pyplot as plt

from pytorch_arap.arap_utils import plot_meshes
from tqdm import tqdm

from mesh_module import Mesh

if torch.cuda.is_available():
	device = "cuda"
else:
	device = "cpu"

class Model(torch.nn.Module):
	"""Verts to be optimised"""

	def __init__(self, meshes, verts, device="cuda"):

		super().__init__()

		self.device = device
		self.verts_template = verts.to(device)
		self.meshes = meshes

		self.verts = torch.nn.Parameter(verts.clone().to(device))

		self.handle_verts = None
		self.handle_pos = None

	def set_target(self, handle_verts, handle_pos):
		self.handle_verts = handle_verts
		self.handle_pos = handle_pos.to(device)

	def forward(self):

		loss_target = torch.nn.functional.mse_loss(self.verts[0,self.handle_verts], self.handle_pos)
		loss_arap = arap_loss(self.meshes, self.verts_template, self.verts, device=self.device)
		loss = loss_target + 0.001 * loss_arap

		return loss

def process_mesh(mesh):

	np.save("vertices.npy", mesh.get_vertices())
	np.save("faces.npy", mesh.get_faces())
	np.save("fixed_vert_indices.npy", mesh.get_fixed_vert_indices())
	np.save("moving_vert_indices.npy", mesh.get_moving_vert_indices())
	np.save("target_positions.npy", mesh.get_target_positions())

	meshes = create_ARAP_meshes(torch.from_numpy(mesh.get_vertices().astype(np.float32)), torch.from_numpy(mesh.get_faces().astype(np.int64)), device=device)
	N = meshes.num_verts_per_mesh()[0]

	# meshes.rotate(mesh_idx=0, rot_x=np.pi/2)

	# # handle as topmost vert
	# handle_verts = [28] # [79]
	# handle_verts = add_one_ring_neighbours(meshes, handle_verts)
	# handle_pos = meshes.verts_padded()[0][handle_verts]
	# handle_pos_shifted = handle_pos.clone()

	# # static as base
	# static_verts = [1792, 3211, 95, 3667] # centres of paws
	# static_verts = add_n_ring_neighbours(meshes, static_verts, n = 6)

	faces = meshes.faces_list()

	# prop = True
	# trisurfs = plot_meshes(ax, meshes.verts_list(), faces, handle_verts=handle_verts, static_verts=static_verts, prop=prop, change_lims=True,
	# 					   color="gray", zoom=1.5)

	# disp_vec = torch.FloatTensor([1, 0, 0])  # displace in x direction

	# disp_frac = 0.6 # fraction of full disp_vec to move in animation
	# step = disp_frac # moves

	# handle_pos_shifted[:] += step * disp_vec

	verts_template = meshes.verts_padded()

	model = Model(meshes, verts_template, device=device)
	# model.set_target(handle_verts, handle_pos_shifted)

	# optimiser = torch.optim.Adam(model.parameters(), lr = 5e-3)

	# n_frames = 50
	# progress = tqdm(total=n_frames)

	# for i in range(n_frames):	
	# 	optimiser.zero_grad()
	# 	loss = model()
	# 	loss.backward()
	# 	optimiser.step()

	# 	progress.n = progress.last_print_n = i+1
	# 	progress.set_description(f"Loss = {loss:.4f}")

	# save_obj("./sad.obj", model.verts[0], faces[0])


def deform_cow():

	vertices = np.load("vertices.npy")
	faces = np.load("faces.npy")
	fixed_vert_indices = np.load("fixed_vert_indices.npy")
	moving_vert_indices = np.load("moving_vert_indices.npy")
	target_positions = np.load("target_positions.npy")

	print(type(vertices))
	print(type(faces))
	print(type(fixed_vert_indices))
	print(type(moving_vert_indices))
	print(type(target_positions))

	meshes = create_ARAP_meshes(torch.from_numpy(vertices.astype(np.float32)), torch.from_numpy(faces.astype(np.int64)), device=device)
	N = meshes.num_verts_per_mesh()[0]

	meshes.rotate(mesh_idx=0, rot_x=np.pi/2)

	# handle as topmost vert
	handle_verts = moving_vert_indices.tolist()
	handle_pos = meshes.verts_padded()[0][handle_verts]
	handle_pos_shifted = target_positions

	# static as base
	static_verts = fixed_vert_indices.tolist()

	faces = meshes.faces_list()

	prop = True
	trisurfs = plot_meshes(ax, meshes.verts_list(), faces, handle_verts=handle_verts, static_verts=static_verts, prop=prop, change_lims=True,
						   color="gray", zoom=1.5)

	verts_template = meshes.verts_padded()

	model = Model(meshes, verts_template, device=device)
	model.set_target(handle_verts, handle_pos_shifted)

	optimiser = torch.optim.Adam(model.parameters(), lr = 5e-3)

	n_frames = 50
	progress = tqdm(total=n_frames)

	for i in range(n_frames):	
		optimiser.zero_grad()
		loss = model()
		loss.backward()
		optimiser.step()

		progress.n = progress.last_print_n = i+1
		progress.set_description(f"Loss = {loss:.4f}")

	save_obj("./sad.obj", model.verts[0], faces[0])




def fib(obj: Mesh):
    vertices = obj.get_vertices()
    faces = obj.get_faces()
    print(vertices.shape)
    print(faces)