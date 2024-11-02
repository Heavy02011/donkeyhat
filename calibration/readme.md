# Calibration

## Steering
- right:  ```echo -e "1000,1500" > /dev/ttyAMA0```
- middle: ```echo -e "1500,1500" > /dev/ttyAMA0```
- left:   ```echo -e "2000,1500" > /dev/ttyAMA0```

## Thottle
- forward: ```echo -e "1500,1600" > /dev/ttyAMA0``` # 1700 "very fast"
- stop:    ```echo -e "1500,1500" > /dev/ttyAMA0```
