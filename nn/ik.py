import math
L1 = 0.23
L2 = 0.23
L1s = L1*L1
L2s = L2*L2

while True:
  x = float(input("x: "))
  y = float(input("y: "))
  ds = math.sqrt(x*x + y*y)

  t0a = math.atan2(y,x)
  z = L1s + ds*ds -L2s
  print(z)
  t0b = math.acos((z)/(2*ds*L2))
  t0 = t0a- t0b
  t1 = math.acos((L1s + L2s - ds*ds)/(2*L1*L2))
  print(t0, t1)
  print(t0*(180/3.1416), t1*(180/3.1416))
  
  
