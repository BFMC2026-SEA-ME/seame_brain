# this code is to test jetson temparature
import glob

def read(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except:
        return None

temps = {}
for z in sorted(glob.glob("/sys/devices/virtual/thermal/thermal_zone*")):
    name = read(z + "/type")
    raw  = read(z + "/temp")
    if not name or not raw:
        continue
    temps[name] = float(raw) / 1000.0

print(temps)  # 예: {'CPU-therm': 44.0, 'GPU-therm': 41.5, ...}

'''
team1@team1-desktop:~/cmh/seame_brain/Brain$ python3 test.py
{'cpu-thermal': 43.531, 'gpu-thermal': 42.937, 'soc0-thermal': 41.218, 
'soc1-thermal': 41.843, 'soc2-thermal': 42.531, 'tj-thermal': 43.531}
'''