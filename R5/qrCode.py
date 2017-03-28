#import sys, qrcode



import qrtools
qr = qrtools.QR()
qr.decode("horn.png")
print qr.data
