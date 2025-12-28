#!/usr/bin/env python3
import cv2
import numpy as np
from pathlib import Path

out_dir = Path(__file__).resolve().parents[1] / 'docs' / 'screenshots'
out_dir.mkdir(parents=True, exist_ok=True)

# Create a synthetic color image (scene)
img = np.full((480,640,3), 200, dtype=np.uint8)
# draw some background shapes
cv2.rectangle(img, (50,50), (200,120), (120,120,160), -1)
cv2.circle(img, (400,300), 60, (0,0,255), -1)  # red object

# create mask (simulate HSV threshold result)
mask = np.zeros((480,640), dtype=np.uint8)
cv2.circle(mask, (400,300), 60, 255, -1)

# find centroid
M = cv2.moments(mask)
if M['m00'] > 0:
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
else:
    cx, cy = 0, 0

# overlay detection on color image
vis = img.copy()
cv2.circle(vis, (cx, cy), 8, (0,255,0), -1)
cv2.circle(vis, (cx, cy), 60, (0,255,0), 2)
cv2.putText(vis, 'Detected object', (cx-80, cy-70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

# save images
cv2.imwrite(str(out_dir / 'tracking_view.png'), vis)
cv2.imwrite(str(out_dir / 'mask_view.png'), mask)

print('Generated screenshots in', out_dir)
