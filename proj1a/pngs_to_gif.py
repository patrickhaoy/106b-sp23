import cv2
import os
import glob
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', '-d', type=str, default=None)
    args = parser.parse_args()

    # Create the frames
    frames = []
    imgs = glob.glob(f"{args.dir}/*.png")
    imgs = sorted(imgs)

    frame = cv2.imread(imgs[0])
    height, width, layers = frame.shape

    video = cv2.VideoWriter(os.path.join(args.dir, 'video.mp4'), cv2.VideoWriter_fourcc(*'MP4V'), 25, (width,height))

    for img in imgs:
        video.write(cv2.imread(img))
    
    cv2.destroyAllWindows()
    video.release()