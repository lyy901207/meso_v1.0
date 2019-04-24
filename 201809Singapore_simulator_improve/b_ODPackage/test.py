'''
Created on 2018/08/04

@author: gong
'''
a=[]
for i in range(0,10):
    a.append(i)
print "print a"
print a

for i in range(0,2):
    a.append(i)
print " print new a"
print a

b=[]
for j in range(7,15):
    b.append(j)
print " print b "
print b

c=[100]
for k in range(0,5):
    c.append(k)
print " print c"
print c

d=[]
# combine two list
print "print results related combination among lists"
print list(set(a).union(set(b)))
print list(set(b).union(set(a)))
print list(set(c).union(set(a)))
a=list(set(a).union(set(c)))
print a
# exclude one list from another one
print "print results related difference among lists"
print list(set(b).difference(set(a)))
print list(set(a).difference(set(b)))
print list(set(a).difference(set(c)))
print list(set(c).difference(set(a)))
