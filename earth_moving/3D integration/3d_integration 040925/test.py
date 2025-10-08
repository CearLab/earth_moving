from coordinate_converter import CoordinateConverter
conv = CoordinateConverter()
print(conv.convert_3d_to_2d(*conv.convert_2d_to_3d(5, 7)))
