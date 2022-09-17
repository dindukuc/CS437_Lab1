import sys
from stopsign import stop_sign_detection as st


while True:
    if st.detect_stop_sign(1) == True:
        print("detected stop sign!")
    else:
        print("No stop sign detected")
