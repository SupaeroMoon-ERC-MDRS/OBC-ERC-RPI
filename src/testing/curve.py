import matplotlib.pyplot as plt
import numpy as np

ThumbDeadZone = 20.0
ThumbCenter = 127.0

def thumb_curve(x):
    if x > ThumbCenter + ThumbDeadZone:
        return (1.3 ** ((x - ThumbCenter - ThumbDeadZone) / 10) - 1) / (1.3 ** ((ThumbCenter - ThumbDeadZone) / 10) - 1)
    elif x < ThumbCenter - ThumbDeadZone:
        return -(1.3 ** ((ThumbCenter - x - ThumbDeadZone) / 10) - 1) / (1.3 ** ((ThumbCenter - ThumbDeadZone) / 10) - 1)
    else:
        return 0.0
    
xx = np.arange(256)
yy = []

for x in xx:
    yy.append(thumb_curve(x))

plt.plot(xx, yy)
plt.show()