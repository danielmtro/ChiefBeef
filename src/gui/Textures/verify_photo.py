from PIL import Image

def resize_image(input_path, output_path, new_width, new_height):
    # Open the image
    with Image.open(input_path) as img:
        # Resize the image
        resized_img = img.resize((new_width, new_height), Image.ANTIALIAS)
        
        # Save the resized image
        resized_img.save(output_path)
        print(f"Image successfully resized to {new_width}x{new_height} and saved to {output_path}")

new_width, new_height = 60, 50

# File paths and target resolution
input_image_path = "question_mark_full_size.png"   # Replace with your input image path
output_image_path = "question_mark.png" # Replace with your desired output path
# Resize the imagepython3 
resize_image(input_image_path, output_image_path, new_width, new_height)


new_width, new_height = 45, 45
input_image_path = "eggplant_full_size.png"   # Replace with your input image path
output_image_path = "eggplant.png" # Replace with your desired output path
# Resize the imagepython3 
resize_image(input_image_path, output_image_path, new_width, new_height)


new_width, new_height = 60, 60
input_image_path = "peach_full_size.png"   # Replace with your input image path
output_image_path = "peach.png" # Replace with your desired output path
# Resize the imagepython3 
resize_image(input_image_path, output_image_path, new_width, new_height)


# with Image.open(input_image_path) as img:
#     print(img.width, img.height)