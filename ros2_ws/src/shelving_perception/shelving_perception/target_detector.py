'''Return the robot's current scan position when that position is empty.'''


class TargetDetector:
    def __init__(self, scan_radius=0.15):
        if scan_radius <= 0:
            raise ValueError('scan_radius must be positive')
        self.scan_radius = float(scan_radius)

    def process(self, detected_books, scan_xyz):
        """Return ``scan_xyz`` when no detected book occupies the scan point.

        The robot is responsible for moving the camera to each scan position.
        Therefore this class does not search the whole shelf or choose a shelf
        row; it only validates the position currently being scanned.
        """
        if scan_xyz is None:
            return None

        x, y, z = map(float, scan_xyz)
        for book in detected_books:
            book_x, book_y, book_z = map(float, book['xyz'])
            distance_squared = (
                (book_x - x) ** 2
                + (book_y - y) ** 2
                + (book_z - z) ** 2
            )
            if distance_squared <= self.scan_radius ** 2:
                return None

        return {
            'xyz': (x, y, z),
            'type': 'empty_shelf_position',
        }