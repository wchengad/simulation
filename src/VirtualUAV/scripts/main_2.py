import sys
import os
from os.path import join, exists
import numpy as np
import cv2

import time
start_time = None


def log_message(message):
    elapsed_time = time.time() - start_time
    print("[%.2f s] %s" % (elapsed_time, message))


if __name__ == '__main__':
    # time logging
    # global start_time
    start_time = time.time()

    from pickle import load
    import argparse

    # parse commandline arguments
    log_message(sys.argv)
    parser = argparse.ArgumentParser(description='Generate synth dataset images.')
    parser.add_argument('--idx', type=int,
                        help='idx of the requested sequence')
    parser.add_argument('--ishape', type=int,
                        help='requested cut, according to the stride')
    parser.add_argument('--stride', type=int,
                        help='stride amount, default 50')
    parser.add_argument('--seq_name', type=str,
                        help='name of the requested sequence')

    args = parser.parse_args(sys.argv[sys.argv.index("--idx"):])

    idx = args.idx
    ishape = args.ishape
    stride = args.stride
    seq_name = args.seq_name


    if idx is None:
        exit(1)
    if ishape is None:
        exit(1)
    if stride is None:
        log_message("WARNING: stride not specified, using default value 50")
        stride = 50

    # import idx info (name, split)
    idx_info = load(open("pkl/idx_info.pickle", 'rb'))

    # add by chengwei: get index for given pose sequence name
    if seq_name is None:
        log_message("WARNING: stride not specified, using default value 50")
    else:
        for index in range(len(idx_info)):
            if idx_info[index]['name'] == seq_name.replace('&', ' '):
                idx = index
    # get runpass
    (runpass, idx) = divmod(idx, len(idx_info))
    log_message("start part 2")

    # import configuration
    import config

    params = config.load_file('config', 'SYNTH_DATA')

    resy = params['resy']
    resx = params['resx']
    tmp_path = params['tmp_path']
    openexr_py2_path = params['openexr_py2_path']
    output_path = params['output_path']

    # check whether openexr_py2_path is loaded from configuration file
    if 'openexr_py2_path' in locals() or 'openexr_py2_path' in globals():
        for exr_path in openexr_py2_path.split(':'):
            sys.path.insert(1, exr_path)

    # to read exr imgs
    import OpenEXR
    import array
    import Imath

    tmp_path = join(tmp_path, '%s' % seq_name.replace("&", ""))
    res_paths = {k: tmp_path for k in params['output_types'] if params['output_types'][k]}
    output_path = join(output_path, 'run%d/%s' % (runpass, seq_name.replace("&", "")))

    if not exists(output_path):
        os.mkdir(output_path)
    if not exists(output_path+"/view"):
        os.mkdir(output_path+"/view")


    FLOAT = Imath.PixelType(Imath.PixelType.FLOAT)

    for file_name in os.listdir(res_paths['depth']):
        if file_name.endswith('.exr'):
            exr_file = OpenEXR.InputFile(join(res_paths['depth'], file_name))
            depth_img = np.reshape([array.array('f', exr_file.channel(Chan, FLOAT)).tolist() for Chan in ("R")],
                       (resx, resy))*1000
            # print(depth_img)
            depth_img = depth_img.astype("uint16")
            depth_file = join(output_path, file_name[-(len("Image%4d_%4d.exr" % (1, 1))):-3] + 'png')
            view_file = join(output_path + "/view", file_name[-(len("Image%4d_%4d.exr" % (1, 1))):-3] + 'png')
            cv2.imwrite(depth_file, depth_img)
            view_img = depth_img.astype("double") / (depth_img.max()) * 255
            cv2.imwrite(view_file, view_img.astype("uint8"))
            print('Writing %s' % depth_file)

    # cleaning up tmp
    if tmp_path != "" and tmp_path != "/":
        log_message("Cleaning up tmp")
        os.system('rm -rf %s' % tmp_path)

    log_message("Completed batch")
