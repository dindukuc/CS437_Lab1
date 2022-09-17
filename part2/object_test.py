import sys
import stop_sign_detection as st




def detect():
    while True:
        if st.detect_stop_sign(1) == True:
            print("detected stop sign!")
        else:
            print("No stop sign detected")




if __name__ == "__main__":
    
    while True:
        try:
            if st.detect_stop_sign(1) == True:
                print("DETECTED STOP SIGN!")
            else:
                print("No stop sign detected")
        except:
            print()
