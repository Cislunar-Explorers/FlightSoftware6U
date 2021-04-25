import argparse
import OpticalNavigation.core.camera as camera
# Run with restart option and then manually restart with sudo reboot
# To record, specify the name of file you wish to create


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", "--mode", help="Restart mode for camera mux or regular run")
    args = vars(ap.parse_args())
    
    if args["mode"] == "restart":
        mux = camera.CameraMux()
        mux.selectCamera(1)
        print("selected mux")
    else:
        mux = camera.CameraMux()
        mux.selectCamera(1)
        print("selected mux")
        cam = camera.Camera()
        f, t = cam.rawObservation(args["mode"])
        print("Recorded video")
        print(f, t)
