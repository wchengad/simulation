import cv2
import argparse
import os
from os.path import join, exists
from scipy import io


def main():
    """
    main function
    :argument dir: directory containing *detph.mat file
    :argument dest: destination directory want to extract depth file
    :argument type: depth file type
    """
    argparser = argparse.ArgumentParser()
    argparser.add_argument("dir", type=str,
                           help="Input a directory containing *depth.mat")
    argparser.add_argument("-v", "--dest", type=str,
                           help="Input a destination directory want to extract depth file")
    argparser.add_argument("-t", "--type", type=str,
                           help="Input the depth file type")
    args = argparser.parse_args()
    mat_dir = args.dir
    if args.dest is None:
        dest = mat_dir+"/depth"
        print(dest)
    else:
        dest = args.dest
    if args.type is None:
        depth_type = "png"
    else:
        depth_type = args.type
    if not exists(dest):
        os.mkdir(dest)
    if not exists(dest+"/view"):
        os.mkdir(dest+"/view")

    for file in os.listdir(mat_dir):
        if file.endswith("depth.mat"):
            depth_images = io.loadmat(join(mat_dir, file))
            idxs = io.whosmat(join(mat_dir, file))

    for idx in idxs:
        depth_img = depth_images[idx[0]]*1000
        depth_img = depth_img.astype("uint16")
        depth_file = join(dest, 'test_'+'{:d}'.format(int(idx[0][6:])-1)+'.'+depth_type)
        view_file = join(dest+"/view", 'test_'+'{:d}'.format(int(idx[0][6:])-1)+'.'+depth_type)
        print('Writing %s' % depth_file)
        cv2.imwrite(depth_file, depth_img)
        view_img = depth_img.astype("double") / (depth_img.max()) * 255
        cv2.imwrite(view_file, view_img.astype("uint8"))


if __name__ == '__main__':
    main()
