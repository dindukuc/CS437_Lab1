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
        time.sleep(1)
        try:
            if st.detect_stop_sign(30) == True:
                print("DETECTED STOP SIGN!")
                exit()
            else:
                print("No stop sign detected")
        except AttributeError:
            print("Caught exception from attribute error!")

    
