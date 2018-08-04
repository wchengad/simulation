import sys
import os
import random
import math
import bpy
import numpy as np

sys.path.append("/usr/local/lib/python3.5/dist-packages")
sys.path.append("/usr/lib/python3/dist-packages")
sys.path.append("/usr/lib/python3.5")
sys.path.append("/home/chengwei/Projects/surreal-master/datageneration")
import cv2
from os.path import join, dirname, realpath, exists
from mathutils import Matrix, Vector, Quaternion, Euler
from random import choice
from pickle import load
from bpy_extras.object_utils import world_to_camera_view as world2cam
import json

sys.path.insert(0, ".")

from time import sleep


def mkdir_safe(directory):
    try:
        os.makedirs(directory)
    except FileExistsError:
        pass


def setState0():
    for ob in bpy.data.objects.values():
        ob.select = False
    bpy.context.scene.objects.active = None


sorted_parts = ['hips', 'leftUpLeg', 'rightUpLeg', 'spine', 'leftLeg', 'rightLeg',
                'spine1', 'leftFoot', 'rightFoot', 'spine2', 'leftToeBase', 'rightToeBase',
                'neck', 'leftShoulder', 'rightShoulder', 'head', 'leftArm', 'rightArm',
                'leftForeArm', 'rightForeArm', 'leftHand', 'rightHand', 'leftHandIndex1', 'rightHandIndex1']
# order
part_match = {'root': 'root', 'bone_00': 'Pelvis', 'bone_01': 'L_Hip', 'bone_02': 'R_Hip',
              'bone_03': 'Spine1', 'bone_04': 'L_Knee', 'bone_05': 'R_Knee', 'bone_06': 'Spine2',
              'bone_07': 'L_Ankle', 'bone_08': 'R_Ankle', 'bone_09': 'Spine3', 'bone_10': 'L_Foot',
              'bone_11': 'R_Foot', 'bone_12': 'Neck', 'bone_13': 'L_Collar', 'bone_14': 'R_Collar',
              'bone_15': 'Head', 'bone_16': 'L_Shoulder', 'bone_17': 'R_Shoulder', 'bone_18': 'L_Elbow',
              'bone_19': 'R_Elbow', 'bone_20': 'L_Wrist', 'bone_21': 'R_Wrist', 'bone_22': 'L_Hand',
              'bone_23': 'R_Hand'}

part2num = {part: (ipart + 1) for ipart, part in enumerate(sorted_parts)}


# create one material per part as defined in a pickle with the segmentation
# this is useful to render the segmentation in a material pass
def create_segmentation(ob, params):
    materials = {}
    vgroups = {}
    with open('/home/chengwei/Projects/surreal-master/datageneration/pkl/segm_per_v_overlap.pkl', 'rb') as f:
        vsegm = load(f)
    bpy.ops.object.material_slot_remove()
    parts = sorted(vsegm.keys())
    for part in parts:
        vs = vsegm[part]
        vgroups[part] = ob.vertex_groups.new(part)
        vgroups[part].add(vs, 1.0, 'ADD')
        bpy.ops.object.vertex_group_set_active(group=part)
        materials[part] = bpy.data.materials['Material'].copy()
        materials[part].pass_index = part2num[part]
        materials[part].use_shadeless = True
        bpy.ops.object.material_slot_add()
        ob.material_slots[-1].material = materials[part]
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='DESELECT')
        bpy.ops.object.vertex_group_select()
        bpy.ops.object.material_slot_assign()
        bpy.ops.object.mode_set(mode='OBJECT')
    return (materials)


# create the different passes that we render
def create_composite_nodes(tree, params):
    # res_paths = {k: join(params['tmp_path'], '%05d_%s' % (idx, k)) for k in params['output_types'] if
    #              params['output_types'][k]}
    res_paths = {k: params['tmp_path'] for k in params['output_types'] if params['output_types'][k]}
    # clear default nodes
    for n in tree.nodes:
        tree.nodes.remove(n)

    # # create node for foreground image
    layers = tree.nodes.new('CompositorNodeRLayers')
    layers.location = -300, 400

    # create node for saving depth
    if (params['output_types']['depth']):
        depth_out = tree.nodes.new('CompositorNodeOutputFile')
        depth_out.location = 40, 700
        depth_out.format.file_format = 'OPEN_EXR'
        # depth_out.use_alpha = False
        depth_out.base_path = res_paths['depth']

    if (params['output_types']['depth']):
        tree.links.new(layers.outputs['Z'], depth_out.inputs[0])  # save depth

    return (res_paths)


# creation of the spherical harmonics material, using an OSL script
def create_sh_material(tree, sh_path, img=None):
    # clear default nodes
    for n in tree.nodes:
        tree.nodes.remove(n)

    uv = tree.nodes.new('ShaderNodeTexCoord')
    uv.location = -800, 400

    uv_xform = tree.nodes.new('ShaderNodeVectorMath')
    uv_xform.location = -600, 400
    uv_xform.inputs[1].default_value = (0, 0, 1)
    uv_xform.operation = 'AVERAGE'

    uv_im = tree.nodes.new('ShaderNodeTexImage')
    uv_im.location = -400, 400
    if img is not None:
        uv_im.image = img

    rgb = tree.nodes.new('ShaderNodeRGB')
    rgb.location = -400, 200

    script = tree.nodes.new('ShaderNodeScript')
    script.location = -230, 400
    script.mode = 'EXTERNAL'
    script.filepath = sh_path  # 'spher_harm/sh.osl' #using the same file from multiple jobs causes white texture
    script.update()

    # the emission node makes it independent of the scene lighting
    emission = tree.nodes.new('ShaderNodeEmission')
    emission.location = -60, 400

    mat_out = tree.nodes.new('ShaderNodeOutputMaterial')
    mat_out.location = 110, 400

    tree.links.new(uv.outputs[2], uv_im.inputs[0])
    tree.links.new(uv_im.outputs[0], script.inputs[0])
    tree.links.new(script.outputs[0], emission.inputs[0])
    tree.links.new(emission.outputs[0], mat_out.inputs[0])


# computes rotation matrix through Rodrigues formula as in cv2.Rodrigues
def Rodrigues(rotvec):
    theta = np.linalg.norm(rotvec)
    r = (rotvec / theta).reshape(3, 1) if theta > 0. else rotvec
    cost = np.cos(theta)
    mat = np.asarray([[0, -r[2], r[1]],
                      [r[2], 0, -r[0]],
                      [-r[1], r[0], 0]])
    return (cost * np.eye(3) + (1 - cost) * r.dot(r.T) + np.sin(theta) * mat)


def init_scene(scene, params, gender='female'):
    # load fbx model
    bpy.ops.import_scene.fbx(
        filepath=join(params['smpl_data_folder'], 'basicModel_%s_lbs_10_207_0_v1.0.2.fbx' % gender[0]),
        axis_forward='Y', axis_up='Z', global_scale=100)
    obname = '%s_avg' % gender[0]
    ob = bpy.data.objects[obname]
    ob.data.use_auto_smooth = False  # autosmooth creates artifacts

    # assign the existing spherical harmonics material
    ob.active_material = bpy.data.materials['Material']

    # delete the default cube (which held the material)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects['Cube'].select = True
    bpy.ops.object.delete(use_global=False)

    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects['Lamp'].select = True
    bpy.ops.object.delete(use_global=False)

    # set camera properties and initial position
    bpy.ops.object.select_all(action='DESELECT')
    cam_ob = bpy.data.objects['Camera']
    scn = bpy.context.scene
    scn.render.engine = 'CYCLES'
    scn.cycles.device = 'GPU'
    scn.objects.active = cam_ob

    # add by chengwei: modify camera rotation
    cam_mat = Matrix(((0., 0., 1, params['camera_distance']),
                      (0., -1, 0., 0),
                      (-1., 0., 0., 0.),
                      (0.0, 0.0, 0.0, 1.0)))

    cam_ob.matrix_world = cam_mat
    # add by chengwei: set camera intrinsic according to Kinect 1's depth camera
    cam_ob.data.angle_x = math.radians(58.5)
    # cam_ob.data.angle_y = math.radians(46.6)
    cam_ob.data.lens_unit = 'FOV'
    cam_ob.data.clip_start = 0.1

    # setup an empty object in the center which will be the parent of the Camera
    # this allows to easily rotate an object around the origin
    scn.cycles.film_transparent = True
    scn.render.layers["RenderLayer"].use_pass_vector = True
    scn.render.layers["RenderLayer"].use_pass_normal = True
    scene.render.layers['RenderLayer'].use_pass_emit = True
    scene.render.layers['RenderLayer'].use_pass_emit = True
    scene.render.layers['RenderLayer'].use_pass_material_index = True

    # set render size
    scn.render.resolution_x = params['resy']
    scn.render.resolution_y = params['resx']
    scn.render.resolution_percentage = 100
    scn.render.image_settings.file_format = 'PNG'
    scn.render.threads_mode = 'FIXED'
    scn.render.threads = 8

    # clear existing animation data
    ob.data.shape_keys.animation_data_clear()
    arm_ob = bpy.data.objects['Armature']
    arm_ob.animation_data_clear()

    return (ob, obname, arm_ob, cam_ob)


# transformation between pose and blendshapes
def rodrigues2bshapes(pose):
    rod_rots = np.asarray(pose).reshape(24, 3)
    mat_rots = [Rodrigues(rod_rot) for rod_rot in rod_rots]
    bshapes = np.concatenate([(mat_rot - np.eye(3)).ravel()
                              for mat_rot in mat_rots[1:]])
    return (mat_rots, bshapes)


# apply trans pose and shape to character
def apply_trans_pose_shape(trans, pose, shape, ob, arm_ob, obname, scene, cam_ob, frame=None):
    # transform pose into rotation matrices (for pose) and pose blendshapes
    mrots, bsh = rodrigues2bshapes(pose)

    # set the location of the first bone to the translation parameter
    arm_ob.pose.bones[obname + '_Pelvis'].location = trans #+ Vector([0, -1, 0])     # by subtracting a vector, to set the pelvis to the origin
    # arm_ob.pose.bones['m_avg_Pelvis'].location = Vector([0, -1, 0])
    if frame is not None:
        arm_ob.pose.bones[obname + '_root'].keyframe_insert('location', frame=frame)
        # print("Not None")
    # set the pose of each bone to the quaternion specified by pose
    for ibone, mrot in enumerate(mrots):
        bone = arm_ob.pose.bones[obname + '_' + part_match['bone_%02d' % ibone]]
        bone.rotation_quaternion = Matrix(mrot).to_quaternion()
        if frame is not None:
            bone.keyframe_insert('rotation_quaternion', frame=frame)
            bone.keyframe_insert('location', frame=frame)

    # apply pose blendshapes
    for ibshape, bshape in enumerate(bsh):
        ob.data.shape_keys.key_blocks['Pose%03d' % ibshape].value = bshape
        if frame is not None:
            ob.data.shape_keys.key_blocks['Pose%03d' % ibshape].keyframe_insert('value', index=-1, frame=frame)

    # apply shape blendshapes
    for ibshape, shape_elem in enumerate(shape):
        ob.data.shape_keys.key_blocks['Shape%03d' % ibshape].value = shape_elem
        if frame is not None:
            ob.data.shape_keys.key_blocks['Shape%03d' % ibshape].keyframe_insert('value', index=-1, frame=frame)

    # arm_ob.pose.bones[obname + '_Pelvis'].location = trans + Vector([0, -1, 0])


# apply trans pose and shape to character
def delete_keyframes(shape, ob, arm_ob, obname, frame=None):

    # arm_ob.pose.bones[obname + '_root'].keyframe_delete('location', frame=frame)
    # set the pose of each bone to the quaternion specified by pose
    for ibone in range(24):
        bone = arm_ob.pose.bones[obname + '_' + part_match['bone_%02d' % ibone]]
        bone.keyframe_delete('rotation_quaternion', frame=frame)
        bone.keyframe_delete('location', frame=frame)

    # apply shape blendshapes
    for ibshape, shape_elem in enumerate(shape):
        ob.data.shape_keys.key_blocks['Shape%03d' % ibshape].keyframe_delete('value', index=-1, frame=frame)

def camera_rotate(cam_ob, content, translation):
#     camera_rotate(cam_ob)
    pre_active_ob = bpy.context.scene.objects.active
    bpy.context.scene.objects.active = cam_ob
    rotation_mat = Matrix.Rotation(float(content[3]), 4, 'Y')
    cam_mat = Matrix(((0., 0., 1, 0),(0., -1, 0., 0),(-1., 0., 0., 0.),(0.0, 0.0, 0.0, 1.0)))
    cam_mat = rotation_mat * cam_mat
    cam_mat[0][3] = float(content[0]) + translation[0]
    cam_mat[1][3] = 0.75 - float(content[2]) + translation[1]
    cam_mat[2][3] = -float(content[1]) + translation[2]
    cam_ob.matrix_world = cam_mat
    bpy.context.scene.objects.active = pre_active_ob

def get_bone_locs(obname, arm_ob, scene, cam_ob):
    n_bones = 24
    render_scale = scene.render.resolution_percentage / 100
    render_size = (int(scene.render.resolution_x * render_scale),
                   int(scene.render.resolution_y * render_scale))
    bone_locations_2d = np.empty((n_bones, 2))
    bone_locations_3d = np.empty((n_bones, 3), dtype='float32')

    # obtain the coordinates of each bone head in image space
    for ibone in range(n_bones):
        bone = arm_ob.pose.bones[obname + '_' + part_match['bone_%02d' % ibone]]
        co_2d = world2cam(scene, cam_ob, arm_ob.matrix_world * bone.head)
        co_3d = arm_ob.matrix_world * bone.head
        bone_locations_3d[ibone] = (co_3d.x,
                                    co_3d.y,
                                    co_3d.z)
        bone_locations_2d[ibone] = (round(co_2d.x * render_size[0]),
                                    round(co_2d.y * render_size[1]))
    return (bone_locations_2d, bone_locations_3d)

# reset the joint positions of the character according to its new shape
def reset_joint_positions(orig_trans, shape, ob, arm_ob, obname, scene, cam_ob, reg_ivs, joint_reg):
    # since the regression is sparse, only the relevant vertex
    #     elements (joint_reg) and their indices (reg_ivs) are loaded
    reg_vs = np.empty((len(reg_ivs), 3))  # empty array to hold vertices to regress from
    # zero the pose and trans to obtain joint positions in zero pose
    apply_trans_pose_shape(orig_trans, np.zeros(72), shape, ob, arm_ob, obname, scene, cam_ob)

    # obtain a mesh after applying modifiers
    bpy.ops.wm.memory_statistics()
    # me holds the vertices after applying the shape blendshapes
    me = ob.to_mesh(scene, True, 'PREVIEW')

    # fill the regressor vertices matrix
    for iiv, iv in enumerate(reg_ivs):
        reg_vs[iiv] = me.vertices[iv].co
    bpy.data.meshes.remove(me)

    # regress joint positions in rest pose
    joint_xyz = joint_reg.dot(reg_vs)
    # adapt joint positions in rest pose
    arm_ob.hide = False
    bpy.ops.object.mode_set(mode='EDIT')
    arm_ob.hide = True
    for ibone in range(24):
        bb = arm_ob.data.edit_bones[obname + '_' + part_match['bone_%02d' % ibone]]
        bboffset = bb.tail - bb.head
        bb.head = joint_xyz[ibone]
        bb.tail = bb.head + bboffset
    bpy.ops.object.mode_set(mode='OBJECT')
    return (shape)

# load poses and shapes
def load_body_data(smpl_data, ob, obname, gender='female', idx=0):
    # load MoSHed data from CMU Mocap (only the given idx is loaded)

    # create a dictionary with key the sequence name and values the pose and trans
    cmu_keys = []
    for seq in smpl_data.files:
        if seq.startswith('pose_'):
            cmu_keys.append(seq.replace('pose_', ''))

    name = sorted(cmu_keys)[idx % len(cmu_keys)]
    # print(name)
    cmu_parms = {}
    for seq in smpl_data.files:
        if seq == ('pose_' + name):
            cmu_parms[seq.replace('pose_', '')] = {'poses': smpl_data[seq],
                                                   'trans': smpl_data[seq.replace('pose_', 'trans_')]}

    # compute the number of shape blendshapes in the model
    n_sh_bshapes = len([k for k in ob.data.shape_keys.key_blocks.keys()
                        if k.startswith('Shape')])

    # load all SMPL shapes
    fshapes = smpl_data['%sshapes' % gender][:, :n_sh_bshapes]

    return (cmu_parms, fshapes, name)


# resize and crop background image wrt given resolution
def img_resize_and_crop(img_dir, res_x: int = 240, res_y: int = 320):
    """
    resize and crop background image wrt given resolution
    :param img_dir: background directory
    :param res_y: given output width
    :param res_x: given output height
    :return: temporally saved cropped and resized image directory
    """
    img = cv2.imread(img_dir)
    rat = max(res_y / img.shape[1], res_x / img.shape[0])
    img_size = (int(img.shape[1] * rat), int(img.shape[0] * rat))
    img = cv2.resize(img, img_size, interpolation=cv2.INTER_AREA)
    img = img[int((img.shape[0] - res_x) / 2):int((img.shape[0] - res_x) / 2) + res_x,
          int((img.shape[1] - res_y) / 2):int((img.shape[1] - res_y) / 2) + res_y]
    cv2.imwrite(img_dir[:-4] + "_tmp.jpg", img)

    return img_dir[:-4] + "_tmp.jpg"


import time

start_time = None


def log_message(message):
    elapsed_time = time.time() - start_time
    print("[%.2f s] %s" % (elapsed_time, message))


def main():
    # time logging
    global start_time
    start_time = time.time()

    import argparse

    # parse commandline arguments
    log_message(sys.argv)
    parser = argparse.ArgumentParser(description='Generate synth dataset images.')
    parser.add_argument('--init_camera_pose', type=str,
                        help='Initial camera pose (x, y, z, yaw)')
    parser.add_argument('--output_dir', type=str,
                        help='Output directory')
    parser.add_argument('--resx', type=int,
                        help='Output image resolution X')
    parser.add_argument('--resy', type=int,
                        help='Output image resolution Y')

    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])

    idx = 0
    ishape = 0
    stride = 50
    seq_name = '09_05'
    init_camera_pose_str = args.init_camera_pose
    output_path = args.output_dir
    resx = args.resx
    resy = args.resy

    if init_camera_pose_str is None:
        log_message("WARNING: initial camera pose not specified, using default value 0.0, 0.0, 0.0, 0.0")
        init_camera_pose_str = "0.0,0.0,0.0,0.0"
    if output_path is None:
        log_message("WARNING: output directory not specified, using default value /home/chengwei/Project/Simulator/result")
        output_path = "/home/chengwei/Project/Simulator/result"
    if resx is None:
        log_message("WARNING: resx not specified, using default value 320")
        resx = 320
    if resy is None:
        log_message("WARNING: resy not specified, using default value 240")
        resy = 240

    log_message("input initial camera pose: %s" % init_camera_pose_str)
    log_message("input output_dir: %s" % output_path)
    log_message("input resx: %d" % resx)
    log_message("input resy: %d" % resy)

    init_camera_pose = init_camera_pose_str.split(',')

    # import configuration
    log_message("Importing configuration")
    import config
    params = config.load_file('/home/chengwei/Projects/Simulator/src/VirtualUAV/scripts/renderer_config', 'SYNTH_DATA')

    smpl_data_folder = params['smpl_data_folder']
    smpl_data_filename = params['smpl_data_filename']
    clothing_option = params['clothing_option']  # grey, nongrey or all
    output_types = params['output_types']
    stepsize = params['stepsize']
    clipsize = params['clipsize']
    openexr_py2_path = params['openexr_py2_path']

    idx_info = load(open("/home/chengwei/Projects/surreal-master/datageneration/pkl/idx_info.pickle", 'rb'))

    # print(1)
    # add by chengwei: get index for given pose sequence name
    if seq_name is None:
        log_message("WARNING: stride not specified, using default value 50")
    else:
        for index in range(len(idx_info)):
            if idx_info[index]['name'] == seq_name.replace('&', ' '):
                idx = index
    # print(idx)
    # get runpass
    (runpass, idx) = divmod(idx, len(idx_info))

    # compute number of cuts
    nb_ishape = max(1, int(np.ceil((idx_info['nb_frames'] - (clipsize - stride)) / stride)))
    log_message("Max ishape: %d" % (nb_ishape - 1))

    assert (ishape < nb_ishape)

    # name is set given idx
    name = idx_info['name']
    tmp_path = join(output_path, '%s' % "depth")

    # check if already computed
    #  + clean up existing tmp folders if any
    if exists(tmp_path) and tmp_path != "" and tmp_path != "/":
        os.system('rm -rf %s' % tmp_path)

    # create tmp directory
    if not exists(tmp_path):
        mkdir_safe(tmp_path)
        mkdir_safe(tmp_path + "/view")

    # initialize RNG with seeds from sequence id
    import hashlib
    s = "synth_data:%d:%d:%d" % (idx, runpass, ishape)
    seed_number = int(hashlib.sha1(s.encode('utf-8')).hexdigest(), 16) % (10 ** 8)
    log_message("GENERATED SEED %d from string '%s'" % (seed_number, s))
    random.seed(seed_number)
    np.random.seed(seed_number)

    if (output_types['vblur']):
        vblur_factor = np.random.normal(0.5, 0.5)
        params['vblur_factor'] = vblur_factor

    log_message("Setup Blender")

    # create copy-spher.harm. directory if not exists
    sh_dir = join(tmp_path, '../spher_harm')
    if not exists(sh_dir):
        mkdir_safe(sh_dir)
    sh_dst = join(sh_dir, 'sh_%02d_%05d.osl' % (runpass, idx))
    os.system('cp /home/chengwei/Projects/surreal-master/datageneration/spher_harm/sh.osl %s' % sh_dst)

    # genders = {0: 'female', 1: 'male'}
    # pick random gender
    # gender = choice(genders)
    gender = 'female'

    scene = bpy.data.scenes['Scene']
    scene.render.engine = 'CYCLES'
    bpy.data.materials['Material'].use_nodes = True
    bpy.data.materials['Material'].use_shadeless = True
    scene.cycles.shading_system = True
    scene.use_nodes = True

    log_message("Listing background images")

    # grab clothing names
    log_message("clothing: %s" % clothing_option)
    with open(join(smpl_data_folder, 'textures', '%s_%s.txt' % (gender, idx_info['use_split']))) as f:
        txt_paths = f.read().splitlines()

    # if using only one source of clothing
    if clothing_option == 'nongrey':
        txt_paths = [k for k in txt_paths if 'nongrey' in k]
    elif clothing_option == 'grey':
        txt_paths = [k for k in txt_paths if 'nongrey' not in k]

    # random clothing texture
    # cloth_img_name = choice(txt_paths)
    # cloth_img_name = join(smpl_data_folder, cloth_img_name)
    # cloth_img = bpy.data.images.load(cloth_img_name)

    # random background
    # bg_img_name = choice(nh_txt_paths)[:-1]
    # bg_img = bpy.data.images.load(img_resize_and_crop(bg_img_name, resx, resy))

    log_message("Loading parts segmentation")
    beta_stds = np.load(join(smpl_data_folder, ('%s_beta_stds.npy' % gender)))

    # log_message("Building materials tree")
    mat_tree = bpy.data.materials['Material'].node_tree
    # create_sh_material(mat_tree, sh_dst, cloth_img)
    res_paths = create_composite_nodes(scene.node_tree, params)

    log_message("Loading smpl data")
    smpl_data = np.load(join(smpl_data_folder, smpl_data_filename))

    log_message("Initializing scene")

    ob, obname, arm_ob, cam_ob = init_scene(scene, params, gender)

    setState0()
    ob.select = True
    bpy.context.scene.objects.active = ob
    segmented_materials = False  # True: 0-24, False: expected to have 0-1 bg/fg

    log_message("Creating materials segmentation")
    # create material segmentation
    if segmented_materials:
        materials = create_segmentation(ob, params)
        prob_dressed = {'leftLeg': .5, 'leftArm': .9, 'leftHandIndex1': .01,
                        'rightShoulder': .8, 'rightHand': .01, 'neck': .01,
                        'rightToeBase': .9, 'leftShoulder': .8, 'leftToeBase': .9,
                        'rightForeArm': .5, 'leftHand': .01, 'spine': .9,
                        'leftFoot': .9, 'leftUpLeg': .9, 'rightUpLeg': .9,
                        'rightFoot': .9, 'head': .01, 'leftForeArm': .5,
                        'rightArm': .5, 'spine1': .9, 'hips': .9,
                        'rightHandIndex1': .01, 'spine2': .9, 'rightLeg': .5}
    else:
        materials = {'FullBody': bpy.data.materials['Material']}
        prob_dressed = {'FullBody': .6}

    orig_pelvis_loc = (arm_ob.matrix_world.copy() * arm_ob.pose.bones[obname + '_Pelvis'].head.copy()) #- Vector((-1., 1., 1.))
    orig_cam_loc = cam_ob.location.copy()

    # unblocking both the pose and the blendshape limits
    for k in ob.data.shape_keys.key_blocks.keys():
        bpy.data.shape_keys["Key"].key_blocks[k].slider_min = -10
        bpy.data.shape_keys["Key"].key_blocks[k].slider_max = 10

    log_message("Loading body data")
    # print(idx)
    cmu_parms, fshapes, name = load_body_data(smpl_data, ob, obname, idx=idx, gender=gender)

    log_message("Loaded body data for %s" % name)

    nb_fshapes = len(fshapes)
    if idx_info['use_split'] == 'train':
        fshapes = fshapes[:int(nb_fshapes * 0.8)]
    elif idx_info['use_split'] == 'test':
        fshapes = fshapes[int(nb_fshapes * 0.8):]

    # pick random real body shape
    shape = choice(fshapes)  # +random_shape(.5) can add noise

    scene.objects.active = arm_ob
    orig_trans = np.asarray(arm_ob.pose.bones[obname + '_Pelvis'].location).copy()

    # create output directory
    if not exists(output_path):
        mkdir_safe(output_path)

    # spherical harmonics material needs a script to be loaded and compiled
    scs = []
    for mname, material in materials.items():
        scs.append(material.node_tree.nodes['Script'])
        scs[-1].filepath = sh_dst
        scs[-1].update()


    data = cmu_parms[name]

    if params['starting_point'] > 0:
        fbegin = params['starting_point']
        fend = min(fbegin + stepsize * clipsize, len(data['poses']))
    else:
        fbegin = ishape * stepsize * stride
        fend = min(ishape * stepsize * stride + stepsize * clipsize, len(data['poses']))

    log_message("Computing how many frames to allocate")
    N = len(data['poses'][fbegin:fend:stepsize])
    log_message("Allocating %d frames in mat file" % N)

    # force recomputation of joint angles unless shape is all zeros
    curr_shape = np.zeros_like(shape)
    nframes = len(data['poses'][::stepsize])

    out_dict = []
    curr_shape = reset_joint_positions(orig_trans, shape, ob, arm_ob, obname, scene,
                                       cam_ob, smpl_data['regression_verts'], smpl_data['joint_regressor'])


    frame_id = 0
    pre_frame_id = 0
    simulator_start = False
    ishape = -1
    error_cntr = 0
    while True:
        try:
            with open('/home/chengwei/Projects/Simulator/result/trajectory.txt.active', 'r+') as f:
                content = f.readlines()
                rt = content[-1][:-1].split('\t')
                frame_id = len(content)
                simulator_start = True
        except EnvironmentError:
            error_cntr = error_cntr + 1
            if error_cntr % 50 == 0:
                print("File does not exists! Waiting...")
            error_cntr = error_cntr % 50
            sleep(0.01)

        if (frame_id == pre_frame_id + 1) or (simulator_start == False and ishape == -1):
            print("in")
            ishape = ishape + 1
            # for each clipsize'th frame in the sequence
            get_real_frame = lambda ifr: ifr
            reset_loc = False
            arm_ob.animation_data_clear()
            cam_ob.animation_data_clear()

            # create a keyframe animation with pose, translation, blendshapes and camera motion
            # LOOP TO CREATE 3D ANIMATION
            for seq_frame, (pose, trans) in enumerate(zip(data['poses'][fbegin:fend:stepsize], data['trans'][fbegin:fend:stepsize])):
                scene.frame_set(get_real_frame(seq_frame))
                # apply the translation, pose and shape to the character
                if ishape > 1:
                    apply_trans_pose_shape(Vector(trans - data['trans'][fbegin]), pose, shape, ob, arm_ob, obname,
                                   scene, cam_ob)
                else:
                    apply_trans_pose_shape(Vector(trans - data['trans'][fbegin]), pose, shape, ob, arm_ob, obname, scene, cam_ob, get_real_frame(seq_frame))
                    arm_ob.pose.bones[obname + '_root'].keyframe_insert('rotation_quaternion', frame=get_real_frame(seq_frame))
                scene.update()

                # Bodies centered only in each minibatch of clipsize frames
                if seq_frame == 0 or reset_loc:
                    reset_loc = False
                    new_pelvis_loc = arm_ob.matrix_world.copy() * arm_ob.pose.bones[obname + '_Pelvis'].head.copy()
                    # cam_ob.location = orig_cam_loc.copy() + (new_pelvis_loc.copy() - orig_pelvis_loc.copy())
                    if ishape is 0:
                        rt = init_camera_pose
                    camera_rotate(cam_ob, rt, new_pelvis_loc.copy())
                    orig_cam_loc = cam_ob.location.copy()
                    # cam_ob.keyframe_delete('location', frame=get_real_frame(seq_frame))
                    cam_ob.keyframe_insert('location', frame=get_real_frame(seq_frame))

            for seq_frame, (pose, trans) in enumerate(
                    zip(data['poses'][fbegin:fend:stepsize], data['trans'][fbegin:fend:stepsize])):
                scene.frame_set(get_real_frame(seq_frame))
                iframe = seq_frame
                scene.render.use_antialiasing = False

                log_message("Rendering sequence %d frame %d" % (ishape, seq_frame))

                # disable render output
                logfile = '/dev/null'
                open(logfile, 'a').close()
                old = os.dup(1)
                sys.stdout.flush()
                os.close(1)
                os.open(logfile, os.O_WRONLY)

                # Render
                bpy.ops.render.render(write_still=True)

                # disable output redirection
                os.close(1)
                os.dup(old)
                os.close(old)

                exr_file = cv2.imread(join(tmp_path, 'Image%04d.exr' % get_real_frame(seq_frame)), cv2.IMREAD_UNCHANGED)

                focal_len = 160 / math.tan(math.radians(58.5)/2)
                for i in range(params['resy']):
                    for j in range(params['resx']):
                        x = exr_file[j,i,0]/focal_len*(i-params['resy']/2)
                        y = exr_file[j,i,0]/focal_len*(j-params['resx']/2)
                        exr_file[j,i,0] = math.sqrt(exr_file[j,i,0]*exr_file[j,i,0] - x*x - y*y)
                depth_img = exr_file[:, :, 0] * 1000
                depth_img = depth_img.astype("uint16")
                depth_file = join(tmp_path, 'Image%06d.png' % (frame_id))
                view_file = join(tmp_path + "/view", 'Image%06d.png' % (frame_id))
                cv2.imwrite(depth_file, depth_img)
                view_img = depth_img.astype("double") / (depth_img.max()) * 255
                cv2.imwrite(view_file, view_img.astype("uint8"))
                os.system("rm %s" % join(tmp_path, 'Image%04d.exr' % get_real_frame(seq_frame)))
            pre_frame_id = frame_id
        # if simulator_start:

        # else:
        #     sleep(0.5)
    # os.system("rm %s" % bg_img_name[:-4] + "_tmp.jpg")


if __name__ == '__main__':
    main()

