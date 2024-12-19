import cv2
from pathlib import Path
from tqdm import tqdm


def extract_frames(video_path, output_folder, target_fps=30):
    """
    Extract frames from a video file.

    Args:
        video_path (str): Path to the input video file
        output_folder (str): Path to the folder where frames will be saved
        target_fps (int): Desired frames per second to extract
    """
    output_path = Path(output_folder)
    output_path.mkdir(parents=True, exist_ok=True)

    video = cv2.VideoCapture(video_path)
    if not video.isOpened():
        raise Exception(f"Error: Could not open video file {video_path}")

    original_fps = video.get(cv2.CAP_PROP_FPS)
    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = frame_count / original_fps

    frame_interval = int(original_fps / target_fps)

    print("Video Properties:")
    print(f"Original FPS: {original_fps}")
    print(f"Total Frames: {frame_count}")
    print(f"Duration: {duration:.2f} seconds")

    frame_number = 0
    saved_count = 0

    with tqdm(total=frame_count, desc="Extracting frames", unit="frames") as pbar:
        while True:
            ret, frame = video.read()
            if not ret:
                break

            if frame_number >= 310:
                if frame_number % frame_interval == 0:
                    frame_path = output_path / f"frame_{saved_count:06d}.jpg"
                    cv2.imwrite(str(frame_path), frame)
                    saved_count += 1
                pbar.update(1)
            frame_number += 1

    video.release()
    print(f"\nExtraction complete! Saved {saved_count} frames to {output_folder}")


if __name__ == "__main__":
    video_file = "dataset.mp4"
    output_dir = "dataset"

    try:
        extract_frames(video_file, output_dir, target_fps=29.97)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
