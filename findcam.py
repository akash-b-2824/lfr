import cv2
import time

def scan_cameras(max_index=10):
    print("Scanning for connected cameras...\n")
    
    for index in range(max_index):
        print(f"Checking camera index {index}...")

        # Try to open camera
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)  # use CAP_DSHOW for Windows
        time.sleep(0.2)

        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"  ✅ Camera found at index {index}")
                
                # Resize preview
                preview = cv2.resize(frame, (480, 360))
                cv2.imshow(f"Camera {index}", preview)

                print("  Showing preview... press any key to continue.")
                cv2.waitKey(0)
                cv2.destroyWindow(f"Camera {index}")
            else:
                print(f"  ⚠️ Camera index {index} opened but no frame read.")
        else:
            print(f"  ❌ No camera at index {index}")
        
        cap.release()

    print("\nScan complete.")

if __name__ == "__main__":
    scan_cameras(10)