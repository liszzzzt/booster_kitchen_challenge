"""Calibration checkerboards: one definition shared by the Studio models, the printouts, detection and PnP.

    stand  10 x 7 squares of 40 mm, 30 mm border  (9 x 6 inner corners)  floor stand in front of the robot,
           head moves (make_board_basset.py)
    hand   6 x 5 squares of 25 mm, 15 mm border   (5 x 4 inner corners)  on the back of a hand, arm moves
           (simulation/hand/tools/add_calib_board.py builds it into the robot; keep the numbers equal)

An odd x even square count makes the pattern look different after a 180 deg turn, so the detected
corner order (and the board frame) never flips. Board frame: origin at the first inner corner, x along a
row, y along a column, z into the board. For a printed board, measure the real square size and pass it
with --square to collect.py.
"""
import cv2
import numpy as np


class Board:
    def __init__(self, name, squares, square, margin):
        self.name = name
        self.SQUARES = squares                                    # columns, rows of squares
        self.PATTERN = (squares[0] - 1, squares[1] - 1)           # inner corners (columns, rows)
        self.SQUARE = square                                      # square edge [m]
        self.MARGIN = margin                                      # white border [m], the detector needs it
        self.SIZE = (squares[0] * square + 2 * margin, squares[1] * square + 2 * margin)   # width, height [m]

    def object_points(self, square=None):
        """Inner corners in the board frame, in the order the detector returns them. (N, 3) float32."""
        cols, rows = self.PATTERN
        grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * (square or self.SQUARE)
        return np.hstack([grid, np.zeros((len(grid), 1))]).astype(np.float32)

    def texture(self, px_per_square=64):
        """Board image (grey, uint8): top-left square black, white margin. Pixel size = SIZE exactly."""
        margin = round(px_per_square * self.MARGIN / self.SQUARE)
        cols, rows = self.SQUARES
        img = np.full((rows * px_per_square + 2 * margin, cols * px_per_square + 2 * margin), 255, np.uint8)
        for r in range(rows):
            for c in range(cols):
                if (r + c) % 2 == 0:
                    y, x = margin + r * px_per_square, margin + c * px_per_square
                    img[y:y + px_per_square, x:x + px_per_square] = 0
        return img

    def detect(self, image, fast=False):
        """Inner corners (N, 2) float32 in image pixels, or None. image: BGR or grey. fast: preview only."""
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        if not fast:
            flags |= cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        found, corners = cv2.findChessboardCornersSB(grey, self.PATTERN, flags=flags)
        if not found:
            return None
        corners = corners.reshape(-1, 2).astype(np.float32)
        # the whole board incl. its white border must be in the image: when it is cut off the detector
        # can number the corners from the wrong end (a 180 deg flip of the board frame; seen in tests)
        return corners if self._border_inside(corners, grey.shape) else None

    def _border_inside(self, corners, shape, pad=2.0):
        H = cv2.findHomography(self.object_points(1.0)[:, :2], corners)[0]   # board (in squares) -> pixels
        if H is None:
            return False
        cols, rows = self.PATTERN
        e = self.MARGIN / self.SQUARE + 1.0                                     # first corner -> board edge
        outline = np.array([[-e, -e], [cols - 1 + e, -e], [cols - 1 + e, rows - 1 + e], [-e, rows - 1 + e]])
        px = cv2.perspectiveTransform(outline.reshape(-1, 1, 2), H).reshape(-1, 2)
        h, w = shape[:2]
        return bool(np.all((px >= pad) & (px <= np.array([w - 1 - pad, h - 1 - pad]))))


BOARDS = {
    "stand": Board("stand", (10, 7), 0.04, 0.03),
    "hand": Board("hand", (6, 5), 0.025, 0.015),
}

# module-level names = the stand board (make_board_basset.py and older code)
_stand = BOARDS["stand"]
SQUARES, PATTERN, SQUARE, MARGIN, SIZE = _stand.SQUARES, _stand.PATTERN, _stand.SQUARE, _stand.MARGIN, _stand.SIZE
object_points, texture, detect = _stand.object_points, _stand.texture, _stand.detect
