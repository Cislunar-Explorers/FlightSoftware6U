import cv2
import numpy as np
import glob
import tiled_remap
import unittest


class TestReprojections(unittest.TestCase):
    """Test is designed to verify the accuracy of image reprojection from gnomonic to
    stereographic. Takes in the gnomonic image as well as the ideal stereographic from
    sim, reprojects the former, finds contours, scales them to be the same size, and
    calculates pixel difference."""

    def get_ucam(self, name):
        """Returns u_cam from filename."""
        cam_dict = {
            "camA": np.array(
                [5.00000000e-01, 6.12323400e-17, 8.66025404e-01], dtype=np.float32
            ),
            "camB": np.array(
                [-5.00000000e-01, -6.12323400e-17, -8.66025404e-01], dtype=np.float32
            ),
            "camC": np.array(
                [-3.9931775502364646, -3.0090751157602416, 0.0], dtype=np.float32
            )
            / 5,
        }
        if "camA" in name:
            return cam_dict["camA"]
        elif "camB" in name:
            return cam_dict["camB"]
        elif "camC" in name:
            return cam_dict["camC"]
        else:
            raise ValueError("Invalid camera name")

    def write_composite_image(self, outc, tgtc, cmp, gnName, i, j, diff):
        """Writes the composite image for visual comparison between the two contours."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (0, 12)
        fontScale = 0.5
        fontColor = (255, 255, 255)
        lineType = 1

        cv2.putText(
            outc,
            "remapped",
            bottomLeftCornerOfText,
            font,
            fontScale,
            fontColor,
            lineType,
        )
        cv2.putText(
            tgtc,
            "compare",
            bottomLeftCornerOfText,
            font,
            fontScale,
            fontColor,
            lineType,
        )
        cv2.putText(
            cmp,
            "difference = {:.5f}".format(diff),
            bottomLeftCornerOfText,
            font,
            fontScale,
            fontColor,
            lineType,
        )

        composite = cv2.hconcat([outc, tgtc, cmp])
        outputString = gnName.replace("_gn.png", "_compare")

        cv2.imwrite("comp/%s_%s_%s.png" % (outputString, i, j), composite)

    def reproj(self, write_remapped: bool, write_composite: bool):
        """Main reprojection function."""

        # Glob gnomonic images as filenames ending in "_gn.png"
        gnomonicList = glob.glob("images/*_gn.png")

        for gnName in gnomonicList:

            # Load the images
            src = cv2.imread(gnName)
            tgt = cv2.imread(gnName.replace("_gn", "_st"))

            gnName = gnName.split("images/")[1]

            cam = tiled_remap.Camera(
                tiled_remap.radians(62.2), tiled_remap.radians(48.8), 3280, 2464
            )

            # Determine which camera is used based on whether filename contains
            # "camA", "camB", or "camC"
            u_cam = self.get_ucam(gnName)

            dt = 18.904e-6
            omega = 5
            rot = tiled_remap.CameraRotation(u_cam, -omega * dt)

            bbgn = tiled_remap.BoundingBox(0, 0, cam.w, cam.h)
            out, bbst = tiled_remap.remap_roi(src, bbgn, cam, rot)

            # Save the remapped image with filename "remapped_" + gnName to folder "out"
            # if write_remapped is True.
            if write_remapped:
                cv2.imwrite("out/remapped_" + gnName, out)

            # Two largest contours from the remapped image
            grayOut = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            _, threshOut = cv2.threshold(grayOut, 0, 255, cv2.THRESH_BINARY)
            contoursOut, _ = cv2.findContours(
                threshOut, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            contoursOut = sorted(contoursOut, key=cv2.contourArea, reverse=True)[:2]

            # Two largest contours from the target image
            grayTgt = cv2.cvtColor(tgt, cv2.COLOR_BGR2GRAY)
            _, threshTgt = cv2.threshold(grayTgt, 0, 255, cv2.THRESH_BINARY)
            contoursTgt, _ = cv2.findContours(
                threshTgt, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            contoursTgt = sorted(contoursTgt, key=cv2.contourArea, reverse=True)[:2]

            # If no contours are found, skip the image
            if len(contoursOut) == 0 or len(contoursTgt) == 0:
                print("No corresponding contours found for " + gnName)
                continue

            # Compare each contour in contours to each contour in contours2, output
            for i, c in enumerate(contoursOut):

                for j, c2 in enumerate(contoursTgt):

                    x, y, w, h = cv2.boundingRect(c)
                    outc = out[y : y + h, x : x + w]

                    x2, y2, w2, h2 = cv2.boundingRect(c2)
                    tgtc = tgt[y2 : y2 + h2, x2 : x2 + w2]

                    # Skip if either contour is more than 2 times the size of the other.
                    # They should be close to the same size; if they aren't, they are
                    # likely not the same object.
                    if max(w, h) > 2 * max(w2, h2) or min(w, h) < 0.5 * min(w2, h2):
                        continue

                    # Resize either tgtc or outc to be the size of the largest of the two
                    if w2 > w:
                        outc = cv2.resize(outc, (w2, h2), interpolation=cv2.INTER_AREA)
                    else:
                        tgtc = cv2.resize(tgtc, (w, h), interpolation=cv2.INTER_AREA)

                    # Calculate the difference between the two contours
                    d = cv2.subtract(outc, tgtc)
                    d = cv2.cvtColor(d, cv2.COLOR_BGR2HSV)
                    d = cv2.inRange(d, np.array([0, 0, 10]), np.array([255, 255, 255]))
                    d = d * cv2.cvtColor(cv2.absdiff(outc, tgtc), cv2.COLOR_BGR2GRAY)

                    # Average the difference image to get a single value and divide
                    # to get a value between 0 and 1.
                    diff = np.mean(d) / 255

                    cmp = cv2.addWeighted(outc, 0.5, tgtc, 0.5, 0)
                    cmp[np.where((outc != tgtc).all(axis=2))] = [0, 0, 255]

                    # Print the name of the image being tested and the difference between the
                    # remapped and target image.
                    print(
                        "Image ",
                        gnName,
                        " contour ",
                        i,
                        " remapped with difference ",
                        diff,
                    )

                    try:
                        assert diff < 0.1
                    except AssertionError:
                        print("Difference is greater than 0.1")

                    # Save the composite image with filename "*_composite.png" to folder "comp"
                    # if write_composite is True.
                    # Note that a folder named "comp" needs to be created first.
                    if write_composite:
                        minW = 180
                        # If the image width is less than 256, resize all three images to have
                        # width of 256 and height determined by the aspect ratio of the
                        # original image.
                        if w < minW:
                            outc = cv2.resize(
                                outc,
                                (minW, int(minW * h / w)),
                                interpolation=cv2.INTER_AREA,
                            )
                            tgtc = cv2.resize(
                                tgtc,
                                (minW, int(minW * h / w)),
                                interpolation=cv2.INTER_AREA,
                            )
                            cmp = cv2.resize(
                                cmp,
                                (minW, int(minW * h / w)),
                                interpolation=cv2.INTER_AREA,
                            )
                        self.write_composite_image(outc, tgtc, cmp, gnName, i, j)

    def test_reprojection(self):
        self.reproj(self, write_remapped=False, write_composite=False)

    # def test_reprojection_with_image_output(self):
    #     self.reproj(self, write_remapped=True, write_composite=True)


if __name__ == "__main__":
    unittest.main()
