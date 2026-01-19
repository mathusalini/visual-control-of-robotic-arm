# ----------------------------
# Find corners + track invalid pairs
# ----------------------------
invalid_pairs = []   # list of (left_img, right_img, reason)
valid_pairs = []     # list of (left_img, right_img)

img_size = None

for left_img, right_img in zip(left_images, right_images):

    imgL = cv2.imread(left_img)
    imgR = cv2.imread(right_img)

    # Safety: check images loaded
    if imgL is None or imgR is None:
        reason = "imread failed"
        invalid_pairs.append((left_img, right_img, reason))
        continue

    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    # Save image size once (from first readable pair)
    if img_size is None:
        img_size = grayL.shape[::-1]  # (width, height)

    retL, cornersL = cv2.findChessboardCorners(grayL, chessboard_size, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, chessboard_size, None)

    if not retL or not retR:
        if not retL and not retR:
            reason = "chessboard NOT found in BOTH"
        elif not retL:
            reason = "chessboard NOT found in LEFT"
        else:
            reason = "chessboard NOT found in RIGHT"

        invalid_pairs.append((left_img, right_img, reason))
        continue

    # Sub-pixel refinement
    cornersL = cv2.cornerSubPix(
        grayL, cornersL, (11, 11), (-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    cornersR = cv2.cornerSubPix(
        grayR, cornersR, (11, 11), (-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    # Store points
    objpoints.append(objp)
    imgpointsL.append(cornersL)
    imgpointsR.append(cornersR)
    valid_pairs.append((left_img, right_img))

print("Valid pairs used:", len(objpoints))
print("Invalid pairs:", len(invalid_pairs))

# Print invalid pair list
if invalid_pairs:
    print("\n--- Invalid pairs list ---")
    for l, r, reason in invalid_pairs:
        print(f"{reason} | {l} | {r}")

    # Optional: write to file
    with open("invalid_pairs.txt", "w") as f:
        for l, r, reason in invalid_pairs:
            f.write(f"{reason}\t{l}\t{r}\n")
    print("\nSaved invalid list to invalid_pairs.txt")
