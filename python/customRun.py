from MVMeshRecon.MeshRecon.util.render import Renderer
from MVMeshRecon.MeshRecon.util.func import load_obj, save_obj, save_images_init
from MVMeshRecon.MeshRecon.remesh import calc_vertex_normals
from MVMeshRecon.utils.w3d_utils import load_mv_prediction, make_wonder3D_cameras

from tqdm import tqdm
from MVMeshRecon.MeshRecon.opt import MeshOptimizer
from MVMeshRecon.utils.loss_utils import NormalLoss

def optimize(vertices, faces, ref_images, renderer, weights, remeshing_steps, edge_len_lims=(0.01, 0.1), decay=0.999):
    # optimizer initialization
    opt = MeshOptimizer(vertices, faces, local_edgelen=False, edge_len_lims=edge_len_lims, gain=0.1)
    vertices = opt.vertices

    # normal optimization step
    loss_func = NormalLoss(mask_loss_weights = 1.)
    for i in tqdm(range(remeshing_steps)):
        opt.zero_grad()
        opt._lr *= decay

        normals = calc_vertex_normals(vertices, faces)
        render_normal = renderer.render_normal(vertices, normals, faces)

        loss_expand = 0.5 * ((vertices + normals).detach() - vertices).pow(2).mean()

        # Extract mask and ground truth mask
        mask = render_normal[..., [3]]
        gtmask = ref_images[..., [3]]

        # Compute loss with the mask
        loss = loss_func(render_normal, ref_images, weights=weights, mask=mask, gtmask=gtmask)
        loss_expansion_weight = 0.1
        loss = loss + loss_expansion_weight * loss_expand

        loss.backward()
        opt.step()
        vertices, faces = opt.remesh(poisson=False)

    return vertices.detach(), faces.detach()




vertices, faces = load_obj("./assets/meshes/owl.obj")
normals = calc_vertex_normals(vertices, faces)

mv, proj = make_wonder3D_cameras(cam_type='ortho')
renderer = Renderer(mv, proj, [1024, 1024] )
normals_rendered = renderer.render_normal(vertices, normals, faces)
rgb_rendered = renderer.render_RGB_vclolor(vertices, vertices, faces)

render_path = 'wonder3Dout/render_out'
save_images_init(normals_rendered, render_path, name='normal')
save_images_init(rgb_rendered, render_path, name='rgb')