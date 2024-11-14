import os
from moviepy.editor import VideoFileClip

# Specify the folder containing the GIFs
folder_path = "/home/luffy/ros_ws/data/experiment_results"

# Iterate over all files in the folder
for filename in os.listdir(folder_path):
    if filename.lower().endswith('.gif'):
        gif_path = os.path.join(folder_path, filename)
        
        # Load the GIF file
        gif = VideoFileClip(gif_path)
        
        # Create the MP4 file name (same name as GIF but with .mp4 extension)
        mp4_filename = os.path.splitext(filename)[0] + '.mp4'
        mp4_path = os.path.join(folder_path, mp4_filename)
        
        # Save as MP4
        gif.write_videofile(mp4_path, codec="libx264")
        
        print(f"Converted {filename} to {mp4_filename}")
