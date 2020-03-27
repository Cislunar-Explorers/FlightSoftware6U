# Optical Navigation

* Sun, Moon and Earth detection
* Position, velocity and attitude control
* Timed image acquisition using gyro

## Unit Tests

* Run `python3 -m unittest tests.<MODULE>.<CLASS>.<TEST>` to run a specific test.
* To run all tests in a file, just use `test.<MODULE>`.

## Find Dataset

[Eclipse and Crescent Images](https://cornell.app.box.com/folder/108235110877)

## UKF Dataset

[EM1_3DOF_Trajectory_June_27_2020_3600sec](https://cornell.app.box.com/folder/108235850425)

**Copy all testing datasets into tests/data**

## Issues

1. [] [find.py](find.py) **Earth bias for blue color is too strong; fails to detect under cloud coverage**
    * One issue that comes up a lot is cloud coverage causes false positive moon detections on top of the Earth. 
2. [] [find.py](find.py) **Hard to distinguish between Moon and Sun**
    * The Moon might contain many pixels that are completely white. Further testing is required to obtain differences in whiteness magnitude. For now, the pure white pixels are dropped (Value range is set to 0-254 instead of 255).
3. [] [find.py](find.py) **Eclipses/Crescent Robustness**
4. [] [cam_meas.py](cam_meas.py) **Verify that detected radii are consistent across all images**
    * System gives up after first body of each type is detected. To filter out detection errors, it should verify the detected radius against the detected radii in all other images. The sun radii should be constant, and the Earth and Moon radii should change slightly.
5. [] [cam_meas.py](cam_meas.py) **When one camera doesn't output images**
    * How should the system handle the case where no images are present in the camera acquisition folders? 
    * Possible causes:
        - [acquisition.py](acquisition.py) hasn't finished capturing images, i.e. need to wait
        - There was a camera I/O or hardware failure that could either be recovered by trying again after a certain amount of time, or is permanent and nothing can be done. 
6. [] [cam_meas.py](cam_meas.py) **A body was not found**
    * How should [ukf.py](ukf.py) handle this case?
7. [] [ukf.py](ukf.py) **UKF computation errors?**
    * Verify if this is a possibility
    * If yes, throw an exception where necessary, and catch it in [controller.py](controller.py)
8. [] [controller.py](controller.py) **Deposit position and velocity estimates into global location**