### TO START:
1. Install Mission Control
2. Install MAVproxy (in my case (Windows), installed into MAVProxy dir)
3. Run SITL simulation in Mission Control
4. Install packages
 > pip install -r requirements.txt
5. Run in terminal (.\MAVProxy\mavproxy.exe (Win) or mavproxy.py):
 > .\MAVProxy\mavproxy.exe --master tcp:127.0.0.1:5763 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
6. Run in terminal
 > python copter_script.py