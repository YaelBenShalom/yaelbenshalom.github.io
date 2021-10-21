import qrcode

img = qrcode.make("https://yaelbenshalom.github.io/")
img.save("myQR.png")