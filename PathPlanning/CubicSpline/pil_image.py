from PIL import Image, ImageDraw, ImageFont
# get an image

# make a blank image for the text, initialized to transparent text color
txt = Image.new('RGBA', (300,300), (255,0,255,0))
txt.show()