from PIL import Image

def resize_image(input_path, output_path, new_width, new_height):
    # Open the image
    with Image.open(input_path) as img:
        # Resize the image
        resized_img = img.resize((new_width, new_height), Image.ANTIALIAS)
        
        # Save the resized image
        resized_img.save(output_path)
        print(f"Image successfully resized to {new_width}x{new_height} and saved to {output_path}")

# File paths and target resolution
input_image_path = "scart.png"   # Replace with your input image path
output_image_path = "shopping_cart_small.png" # Replace with your desired output path
new_width, new_height = 40, 30

# Resize the image
resize_image(input_image_path, output_image_path, new_width, new_height)