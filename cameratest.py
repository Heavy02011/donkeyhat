import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from collections import deque

def analyze_motion_blur(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()

def analyze_rolling_shutter(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
    horizontal_lines = 0
    if lines is not None:
        for rho, theta in lines[:, 0]:
            if np.abs(theta - np.pi/2) < 0.1:
                horizontal_lines += 1
    return horizontal_lines

def record_and_analyze_video(duration=10, strobe_on=False):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(f'output_{"strobe" if strobe_on else "normal"}.avi', fourcc, 20.0, (640, 480))

    blur_values = []
    rs_values = []
    start_time = time.time()
    while(time.time() - start_time < duration):
        ret, frame = cap.read()
        if ret:
            if strobe_on:
                if int((time.time() - start_time) * 60) % 2 == 0:
                    frame = np.zeros_like(frame)
            
            blur_value = analyze_motion_blur(frame)
            rs_value = analyze_rolling_shutter(frame)
            blur_values.append(blur_value)
            rs_values.append(rs_value)
            
            out.write(frame)
            cv2.imshow('Recording', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()
    return blur_values, rs_values

def create_comparison_video_with_analysis():
    cap1 = cv2.VideoCapture('output_normal.avi')
    cap2 = cv2.VideoCapture('output_strobe.avi')

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('comparison_with_analysis.avi', fourcc, 20.0, (1280, 720))

    blur_normal = deque(maxlen=20)
    blur_strobe = deque(maxlen=20)
    rs_normal = deque(maxlen=20)
    rs_strobe = deque(maxlen=20)

    while(cap1.isOpened() and cap2.isOpened()):
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        
        if ret1 and ret2:
            # Analyze frames
            blur_normal.append(analyze_motion_blur(frame1))
            blur_strobe.append(analyze_motion_blur(frame2))
            rs_normal.append(analyze_rolling_shutter(frame1))
            rs_strobe.append(analyze_rolling_shutter(frame2))

            # Create plots
            plt.figure(figsize=(12, 4))
            plt.subplot(121)
            plt.plot(list(blur_normal), label='Normal')
            plt.plot(list(blur_strobe), label='Strobe')
            plt.title('Motion Blur Analysis')
            plt.ylabel('Laplacian Variance')
            plt.legend()

            plt.subplot(122)
            plt.plot(list(rs_normal), label='Normal')
            plt.plot(list(rs_strobe), label='Strobe')
            plt.title('Rolling Shutter Artifact Analysis')
            plt.ylabel('Number of Horizontal Lines')
            plt.legend()

            plt.tight_layout()
            
            # Convert plot to image
            plt.savefig('temp_plot.png')
            plt.close()
            plot_img = cv2.imread('temp_plot.png')
            plot_img = cv2.resize(plot_img, (1280, 240))

            # Combine frames and plot
            combined_frame = np.vstack((np.hstack((frame1, frame2)), plot_img))
            
            # Add labels
            cv2.putText(combined_frame, 'Normal', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(combined_frame, 'Strobe', (650, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            out.write(combined_frame)
            
            cv2.imshow('Comparison with Analysis', combined_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap1.release()
    cap2.release()
    out.release()
    cv2.destroyAllWindows()

def main():
    print("Recording and analyzing video without strobe effect...")
    blur_normal, rs_normal = record_and_analyze_video(duration=10, strobe_on=False)
    
    print("Recording and analyzing video with simulated strobe effect...")
    blur_strobe, rs_strobe = record_and_analyze_video(duration=10, strobe_on=True)
    
    print("Creating comparison video with analysis...")
    create_comparison_video_with_analysis()
    
    print("Done! Check 'comparison_with_analysis.avi' for the result.")

    # Plot overall results
    plt.figure(figsize=(12, 6))
    plt.subplot(121)
    plt.boxplot([blur_normal, blur_strobe], labels=['Normal', 'Strobe'])
    plt.title('Overall Motion Blur Analysis')
    plt.ylabel('Laplacian Variance')

    plt.subplot(122)
    plt.boxplot([rs_normal, rs_strobe], labels=['Normal', 'Strobe'])
    plt.title('Overall Rolling Shutter Artifact Analysis')
    plt.ylabel('Number of Horizontal Lines')

    plt.tight_layout()
    plt.savefig('overall_analysis.png')
    plt.close()

    print("Overall analysis saved as 'overall_analysis.png'")

if __name__ == "__main__":
    main()