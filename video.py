#!/usr/bin/env python3
import os
import rosbag
from cv_bridge import CvBridge
import cv2
import shutil
import glob
import subprocess


def extract_images(bag_file, output_dir, image_topic, frame_prefix="frame"):
    os.makedirs(output_dir, exist_ok=True)
    bridge = CvBridge()
    frame_count = 0

    with rosbag.Bag(bag_file, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[image_topic]):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame_path = os.path.join(output_dir, f"{frame_prefix}{frame_count:04d}.jpg")
            cv2.imwrite(frame_path, cv_img)
            frame_count += 1
    return frame_count


def create_video_ffmpeg(image_dir, video_path, fps=30):
    input_pattern = os.path.join(image_dir, "frame%04d.jpg")

    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        input_pattern,
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-preset",
        "medium",
        "-crf",
        "23",
        video_path,
    ]

    try:
        subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error creating video: {e}")
        print(f"FFmpeg error output: {e.stderr.decode('utf-8')}")
        return False
    except FileNotFoundError:
        print("FFmpeg not found. Please ensure FFmpeg is installed and in your PATH.")
        return False


def process_bag_files(input_dir, image_topic):
    if not os.path.exists(input_dir):
        print(f"Input directory '{input_dir}' does not exist!")
        return

    bag_files = glob.glob(os.path.join(input_dir, "*.bag"))

    if not bag_files:
        print(f"No .bag files found in '{input_dir}'")
        return

    for bag_idx, bag_file in enumerate(bag_files, 1):
        bag_name = os.path.basename(bag_file)
        output_folder = f"Bag{bag_idx}"

        os.makedirs(output_folder, exist_ok=True)

        temp_img_dir = os.path.join(output_folder, "temp_frames")
        frame_count = extract_images(bag_file, temp_img_dir, image_topic)

        if frame_count > 0:
            video_name = bag_name.replace(".bag", ".mp4")
            video_path = os.path.join(output_folder, video_name)

            if create_video_ffmpeg(temp_img_dir, video_path):
                print(f"Created video: {video_path}")
            else:
                print(f"Failed to create video for {bag_name}")

        dest_bag_path = os.path.join(output_folder, bag_name)
        shutil.copy2(bag_file, dest_bag_path)
        print(f"Copied bag file to: {dest_bag_path}")

        shutil.rmtree(temp_img_dir)

        print(f"Processed {bag_name}: Extracted {frame_count} frames and created video in {output_folder}/")


if __name__ == "__main__":
    process_bag_files("bag_dir", "zed topic")
