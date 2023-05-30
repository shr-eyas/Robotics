import numpy as np
inp = [1,2,3,4,5,6,7,8]
len = int(len(inp))

if(len%2==0):
    x=int(len/2)
else:
    x=int((len+1)/2)

a = inp[:x]
b = inp[x:len]
c = np.zeros(len)

print(a)
print(b)
i = 0
j = 0

for k in range(len):
    if a[i]<b[j]:
        c[k]=a[i]
        i+=1
    else:
        c[k]=b[j]
        j+=1

print(c)
