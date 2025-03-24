from PIL import Image
import pillow_heif
import re

def convert_heic_to_jpg(path):
    heif_file = pillow_heif.read_heif(path)
    image = Image.frombytes(
        heif_file.mode,
        heif_file.size,
        heif_file.data,
        "raw",
    )
    match = re.search(r"([^\\/]+)[\\/]+([^\\/\.]+)(?=\.[^\\/]+$)", path)
    new_path = f"./imgs/.heic_to_jpg/{match.group(2)}.jpg"
    image.save(new_path, format("jpeg"))
    
    return new_path

# If the image is in heic format then we must convert it before using it with cv2
def check_format(path):
    # TODO: chek if jpg already exists so that you can avoid converting again

    if re.search(r".+\.heic", path):
        path = convert_heic_to_jpg(path) # covnert heic to jpg
    
    return path
    