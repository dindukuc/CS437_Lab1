import stop_sign_detection as st
import time




def detect():
    
    time.sleep(1)
    if st.detect_stop_sign(15) == True:
        print("DETECTED STOP SIGN! EXITING PROGRAM.....")
        exit()
    else:
        print("No stop sign detected")




if __name__ == "__main__":
    
    try:
        while True:
            detect()
    except:
        pass;
        # print("Some error detected...read log.txt")
        # exit()
    
