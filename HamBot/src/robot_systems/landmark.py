class Landmark:
    def __init__(self, x, y, width, height, r, g, b):
        """
        Represents a detected landmark in the camera frame.

        Args:
            x (int): X-coordinate of the center of the landmark.
            y (int): Y-coordinate of the center of the landmark.
            width (int): Width of the bounding box.
            height (int): Height of the bounding box.
            r (int): Red component of detected color.
            g (int): Green component of detected color.
            b (int): Blue component of detected color.
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.r = r
        self.g = g
        self.b = b

    def __repr__(self):
        return f"Landmark(x={self.x}, y={self.y}, w={self.width}, h={self.height}, R={self.r}, G={self.g}, B={self.b})"